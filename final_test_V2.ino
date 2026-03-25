// final_test_V2.ino
// ESP32 BLE Client — connects to PI-3 peripheral
// Parses combined payload: "HR,SPO2,MOTION_STATE,LIN_MAG"
// e.g. "72,98,MOVING,3.45"  |  "68,97,STILL,0.00"
//      "80,96,IMPACT,11.20" |  "75,98,FREEFALL,0.00"
//
// Display (XIAO round screen / LVGL):
//   Top    → HR (bpm) + SpO2 (%)
//   Middle → divider line
//   Bottom → Motion state (colour-coded) + linear accel when moving

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define USE_ARDUINO_GFX_LIBRARY
#ifndef BLACK
#define BLACK 0x0000
#endif

#include <Arduino.h>
#include <lvgl.h>
#include "lv_xiao_round_screen.h"

// ── BLE UUIDs (must match PI-3 peripheral) ───────────────────────────────────
static BLEUUID serviceUUID("6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10");
static BLEUUID charUUID   ("c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11");

// ── BLE handles ───────────────────────────────────────────────────────────────
BLEClient*               pClient              = nullptr;
static BLERemoteCharacteristic* pRemoteChar   = nullptr;
static BLEAdvertisedDevice*     myDevice       = nullptr;
static boolean                  doConnect      = false;
static bool                     ble_initialized = false;
static uint32_t                 last_scan       = 0;

// ── Parsed values ─────────────────────────────────────────────────────────────
volatile int   heartRate    = 0;
volatile int   SpO2         = 0;

// Motion state enum
enum MotionState { MOT_STILL, MOT_MOVING, MOT_IMPACT, MOT_FREEFALL, MOT_UNKNOWN };
volatile MotionState motionState   = MOT_UNKNOWN;
volatile float       motionLinMag  = 0.0f;

// ── Connection / UI flags ─────────────────────────────────────────────────────
boolean        connected_status = false;
static uint32_t last_notify     = 0;
static uint32_t Timeout         = 10000;   // ms
volatile bool  update_UI        = false;

// ── LVGL labels ───────────────────────────────────────────────────────────────
static lv_obj_t* hr_label     = nullptr;
static lv_obj_t* motion_label = nullptr;

// ═════════════════════════════════════════════════════════════════════════════
// UI
// ═════════════════════════════════════════════════════════════════════════════
void build_screen() {
  lv_obj_clean(lv_scr_act());

  // ── HR / SpO2 (upper half) ────────────────────────────────────────────────
  hr_label = lv_label_create(lv_scr_act());
  lv_label_set_text(hr_label, "HR: --\nSpO2: --");
  lv_obj_set_style_text_align(hr_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(hr_label, LV_ALIGN_CENTER, 0, -35);

  // ── Thin divider ──────────────────────────────────────────────────────────
  lv_obj_t* divider = lv_obj_create(lv_scr_act());
  lv_obj_set_size(divider, 160, 2);
  lv_obj_align(divider, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(divider, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(divider, 0, 0);

  // ── Motion (lower half) ───────────────────────────────────────────────────
  motion_label = lv_label_create(lv_scr_act());
  lv_label_set_text(motion_label, "Motion:\n--");
  lv_obj_set_style_text_align(motion_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(motion_label, LV_ALIGN_CENTER, 0, 38);
}

// ═════════════════════════════════════════════════════════════════════════════
// NOTIFY CALLBACK — parses "HR,SPO2,MOTION,LIN_MAG"
// ═════════════════════════════════════════════════════════════════════════════
static void notifyCallback(BLERemoteCharacteristic* chr,
                            uint8_t* data, size_t len, bool isNotify) {
  (void)chr; (void)isNotify;

  std::string s((const char*)data, (const char*)data + len);
  Serial.print("[RX] "); Serial.println(s.c_str());
  last_notify = millis();

  // ── Field 0: HR ───────────────────────────────────────────────────────────
  int c1 = s.find(',');
  if (c1 == (int)std::string::npos) { Serial.println("Parse error: no field 1"); return; }
  int hr = atoi(s.substr(0, c1).c_str());
  if (hr >= 0 && hr <= 250) heartRate = hr;

  // ── Field 1: SpO2 ─────────────────────────────────────────────────────────
  int c2 = s.find(',', c1 + 1);
  if (c2 == (int)std::string::npos) { Serial.println("Parse error: no field 2"); return; }
  int spo2 = atoi(s.substr(c1 + 1, c2 - c1 - 1).c_str());
  if (spo2 >= 0 && spo2 <= 100) SpO2 = spo2;

  // ── Field 2: Motion state ─────────────────────────────────────────────────
  int c3 = s.find(',', c2 + 1);
  if (c3 == (int)std::string::npos) { Serial.println("Parse error: no field 3"); return; }
  std::string motStr = s.substr(c2 + 1, c3 - c2 - 1);

  if      (motStr == "STILL")    motionState = MOT_STILL;
  else if (motStr == "MOVING")   motionState = MOT_MOVING;
  else if (motStr == "IMPACT")   motionState = MOT_IMPACT;
  else if (motStr == "FREEFALL") motionState = MOT_FREEFALL;
  else                           motionState = MOT_UNKNOWN;

  // ── Field 3: Linear magnitude ─────────────────────────────────────────────
  motionLinMag = atof(s.substr(c3 + 1).c_str());

  Serial.printf("  HR=%d  SpO2=%d  Mot=%s  Lin=%.2f\n",
                heartRate, SpO2, motStr.c_str(), (float)motionLinMag);

  update_UI = true;
}

// ═════════════════════════════════════════════════════════════════════════════
// CLIENT CALLBACK
// ═════════════════════════════════════════════════════════════════════════════
class MyClientCallback : public BLEClientCallbacks {
  void onDisconnect(BLEClient*) {
    Serial.println("[BLE] Disconnected");
    connected_status = false;
    update_UI        = true;
  }
};

// ═════════════════════════════════════════════════════════════════════════════
// CONNECT
// ═════════════════════════════════════════════════════════════════════════════
bool connectToServer() {
  Serial.println("[BLE] Connecting…");

  if (pClient != nullptr) {
    if (pClient->isConnected()) pClient->disconnect();
    delete pClient;
    pClient = nullptr;
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println("[BLE] Connection failed");
    return false;
  }

  BLERemoteService* pSvc = pClient->getService(serviceUUID);
  if (!pSvc) {
    Serial.println("[BLE] Service not found");
    pClient->disconnect();
    return false;
  }

  pRemoteChar = pSvc->getCharacteristic(charUUID);
  if (!pRemoteChar) {
    Serial.println("[BLE] Characteristic not found");
    pClient->disconnect();
    return false;
  }

  if (pRemoteChar->canNotify())
    pRemoteChar->registerForNotify(notifyCallback);

  Serial.println("[BLE] Connected and subscribed!");
  connected_status = true;
  update_UI        = true;
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// SCAN CALLBACK
// ═════════════════════════════════════════════════════════════════════════════
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.println("[BLE] Found PI-3!");
      BLEDevice::getScan()->stop();
      if (myDevice) { delete myDevice; myDevice = nullptr; }
      myDevice   = new BLEAdvertisedDevice(advertisedDevice);
      doConnect  = true;
    }
  }
};

// ═════════════════════════════════════════════════════════════════════════════
// SCAN
// ═════════════════════════════════════════════════════════════════════════════
static void ble_start_scan() {
  if (!ble_initialized) {
    BLEDevice::init("");
    ble_initialized = true;
  }
  BLEDevice::getScan()->clearResults();
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);
  pScan->start(5, true);
  last_scan = millis();
}

// ═════════════════════════════════════════════════════════════════════════════
// UI REFRESH
// ═════════════════════════════════════════════════════════════════════════════
void refresh_ui() {
  if (!update_UI) return;
  update_UI = false;

  // ── HR / SpO2 ─────────────────────────────────────────────────────────────
  if (hr_label) {
    char buf[40];
    if (connected_status)
      snprintf(buf, sizeof(buf), "HR: %d bpm\nSpO2: %d%%", heartRate, SpO2);
    else
      snprintf(buf, sizeof(buf), "HR: --\nSpO2: --");
    lv_label_set_text(hr_label, buf);
  }

  // ── Motion ────────────────────────────────────────────────────────────────
  if (motion_label) {
    char buf[60];
    if (!connected_status) {
      snprintf(buf, sizeof(buf), "Motion:\n--");
      lv_obj_set_style_text_color(motion_label, lv_color_hex(0x888888), 0);
    } else {
      switch (motionState) {

        case MOT_STILL:
          snprintf(buf, sizeof(buf), "Motion:\nSTILL");
          lv_obj_set_style_text_color(motion_label, lv_color_hex(0x00CC00), 0);
          break;

        case MOT_MOVING:
          // Show linear accel value when moving
          if ((float)motionLinMag > 0.01f)
            snprintf(buf, sizeof(buf), "Motion:\nMOVING\n%.1f m/s\xB2", (float)motionLinMag);
          else
            snprintf(buf, sizeof(buf), "Motion:\nMOVING");
          lv_obj_set_style_text_color(motion_label, lv_color_hex(0x00AAFF), 0);
          break;

        case MOT_IMPACT:
          snprintf(buf, sizeof(buf), "Motion:\nIMPACT!\n%.1f m/s\xB2", (float)motionLinMag);
          lv_obj_set_style_text_color(motion_label, lv_color_hex(0xFF2200), 0);
          break;

        case MOT_FREEFALL:
          snprintf(buf, sizeof(buf), "Motion:\nFREE-FALL!");
          lv_obj_set_style_text_color(motion_label, lv_color_hex(0xFF8800), 0);
          break;

        default:
          snprintf(buf, sizeof(buf), "Motion:\n--");
          lv_obj_set_style_text_color(motion_label, lv_color_hex(0x888888), 0);
          break;
      }
    }
    lv_label_set_text(motion_label, buf);
    lv_obj_invalidate(motion_label);
  }

  lv_refr_now(NULL);
}

// ═════════════════════════════════════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  lv_init();
  lv_xiao_disp_init();
  lv_xiao_touch_init();
  build_screen();
  Serial.println("[ESP32] Starting BLE scan for PI-3…");
  ble_start_scan();
}

// ═════════════════════════════════════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  lv_timer_handler();
  delay(5);

  // ── Connect if device found ───────────────────────────────────────────────
  if (doConnect) {
    connectToServer();
    doConnect = false;
  }

  // ── Timeout / disconnection handling ─────────────────────────────────────
  if (pClient != nullptr && pClient->isConnected()) {
    if (last_notify != 0 && (millis() - last_notify) > Timeout) {
      last_notify      = 0;
      connected_status = false;
      Serial.println("[BLE] Notify timeout — disconnecting");
      pClient->disconnect();
      update_UI = true;
    } else {
      connected_status = true;
    }
  } else if (!doConnect) {
    if (connected_status) {
      Serial.println("[BLE] Disconnection detected (poll)");
      connected_status = false;
      update_UI        = true;
    }
    if (millis() - last_scan > 6000) {
      Serial.println("[BLE] Scanning…");
      ble_start_scan();
    }
  }

  // ── Refresh display ───────────────────────────────────────────────────────
  refresh_ui();
}
