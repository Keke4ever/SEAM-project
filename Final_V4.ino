// Final_V4.ino
// ─────────────────────────────────────────────────────────────────────────────
// Role 1 : BLE Client  → connects to PI-3 peripheral
//          Parses payload: "HR,SPO2,ACTIVITY,FALL_STATE"
//          ACTIVITY   : STILL | WALKING | RUNNING | JUMPING | MOVING
//          FALL_STATE : NORMAL | FALL | SLIP_FALL
//          e.g. "72,98,WALKING,NORMAL"
//               "65,96,MOVING,FALL"
//               "70,97,MOVING,SLIP_FALL"
//
// Role 2 : BLE Server/Peripheral → Mobile / Web App
//          App subscribes to notify characteristic to receive live vitals.
//          App can write commands (PING, REQ_DATA, STATUS) to write char.
//
// Role 3 : LVGL round display (XIAO round screen)
//          Top    → HR (bpm) + SpO2 (%)
//          Middle → divider line
//          Bottom → Activity (colour-coded) + fall/slip alert
//
// Colour coding:
//   STILL    → green  (#00CC00)
//   WALKING  → cyan   (#00DDFF)
//   RUNNING  → orange (#FF8800)
//   JUMPING  → yellow (#FFDD00)
//   MOVING   → blue   (#00AAFF)
//   FALL     → red    (#FF2200)  [overrides activity colour]
//   SLIP_FALL→ magenta(#FF00CC)  [overrides activity colour]
//
// Data flow:
//   PI-3  ──BLE notify──►  ESP32  ──LVGL──►  Round Display
//                              └──BLE notify──►  Mobile / Web App
//                              ◄──BLE write───   Mobile / Web App
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

#define USE_ARDUINO_GFX_LIBRARY
#ifndef BLACK
#define BLACK 0x0000
#endif

#include <lvgl.h>
#include "lv_xiao_round_screen.h"

// ═════════════════════════════════════════════════════════════════════════════
//  UUIDs
// ═════════════════════════════════════════════════════════════════════════════
static BLEUUID PI_SERVICE_UUID("6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10");
static BLEUUID PI_CHAR_UUID   ("c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11");

#define ESP_SERVICE_UUID     "87654321-4321-4321-4321-ba0987654321"
#define ESP_NOTIFY_CHAR_UUID "11111111-2222-3333-4444-555555555555"
#define ESP_WRITE_CHAR_UUID  "66666666-7777-8888-9999-aaaaaaaaaaaa"

// ═════════════════════════════════════════════════════════════════════════════
//  BLE handles — CLIENT (ESP32 → PI-3)
// ═════════════════════════════════════════════════════════════════════════════
static BLEClient*               pClient       = nullptr;
static BLERemoteCharacteristic* pRemoteChar   = nullptr;
static BLEAdvertisedDevice*     myDevice      = nullptr;
static bool                     doConnect     = false;
static bool                     ble_initialized = false;
static uint32_t                 last_scan     = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  BLE handles — SERVER (ESP32 → App)
// ═════════════════════════════════════════════════════════════════════════════
static BLEServer*         pServer         = nullptr;
static BLECharacteristic* pNotifyChar     = nullptr;
static BLECharacteristic* pWriteChar      = nullptr;
static bool               appConnected    = false;

// ═════════════════════════════════════════════════════════════════════════════
//  Parsed vitals
// ═════════════════════════════════════════════════════════════════════════════
volatile int   heartRate = 0;
volatile int   SpO2      = 0;

// Activity states
enum ActivityState {
  ACT_STILL,
  ACT_WALKING,
  ACT_RUNNING,
  ACT_JUMPING,
  ACT_MOVING,
  ACT_UNKNOWN
};

// Fall states
enum FallState {
  FALL_NORMAL,
  FALL_DETECTED,
  FALL_SLIP
};

volatile ActivityState activityState = ACT_UNKNOWN;
volatile FallState     fallState     = FALL_NORMAL;

String latestPayload = "HR:0,SPO2:0,ACT:--,FALL:NORMAL";

// ═════════════════════════════════════════════════════════════════════════════
//  Connection / UI flags
// ═════════════════════════════════════════════════════════════════════════════
static bool     connected_status = false;
static uint32_t last_notify      = 0;
static uint32_t Timeout          = 10000;
volatile bool   update_UI        = false;

// ═════════════════════════════════════════════════════════════════════════════
//  LVGL labels
// ═════════════════════════════════════════════════════════════════════════════
static lv_obj_t* hr_label       = nullptr;
static lv_obj_t* activity_label = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void forwardToApp(const String& payload);

// ═════════════════════════════════════════════════════════════════════════════
//  LVGL UI
// ═════════════════════════════════════════════════════════════════════════════
void build_screen() {
  lv_obj_clean(lv_scr_act());

  // HR / SpO2 (upper half)
  hr_label = lv_label_create(lv_scr_act());
  lv_label_set_text(hr_label, "HR: --\nSpO2: --");
  lv_obj_set_style_text_align(hr_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(hr_label, LV_ALIGN_CENTER, 0, -35);

  // Thin divider
  lv_obj_t* divider = lv_obj_create(lv_scr_act());
  lv_obj_set_size(divider, 160, 2);
  lv_obj_align(divider, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(divider, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(divider, 0, 0);

  // Activity / fall (lower half)
  activity_label = lv_label_create(lv_scr_act());
  lv_label_set_text(activity_label, "Activity:\n--");
  lv_obj_set_style_text_align(activity_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(activity_label, LV_ALIGN_CENTER, 0, 38);
}

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

  // ── Activity / Fall label ─────────────────────────────────────────────────
  if (activity_label) {
    char buf[60];

    if (!connected_status) {
      snprintf(buf, sizeof(buf), "Activity:\n--");
      lv_obj_set_style_text_color(activity_label, lv_color_hex(0x888888), 0);

    } else if (fallState == FALL_DETECTED) {
      // Fall takes priority — show in red regardless of activity
      snprintf(buf, sizeof(buf), "FALL\nDETECTED!");
      lv_obj_set_style_text_color(activity_label, lv_color_hex(0xFF2200), 0);

    } else if (fallState == FALL_SLIP) {
      // Slip-fall — show in magenta
      snprintf(buf, sizeof(buf), "SLIP\nFALL!");
      lv_obj_set_style_text_color(activity_label, lv_color_hex(0xFF00CC), 0);

    } else {
      // Normal activity display
      switch (activityState) {
        case ACT_STILL:
          snprintf(buf, sizeof(buf), "Activity:\nSTILL");
          lv_obj_set_style_text_color(activity_label, lv_color_hex(0x00CC00), 0);
          break;

        case ACT_WALKING:
          snprintf(buf, sizeof(buf), "Activity:\nWALKING");
          lv_obj_set_style_text_color(activity_label, lv_color_hex(0x00DDFF), 0);
          break;

        case ACT_RUNNING:
          snprintf(buf, sizeof(buf), "Activity:\nRUNNING");
          lv_obj_set_style_text_color(activity_label, lv_color_hex(0xFF8800), 0);
          break;

        case ACT_JUMPING:
          snprintf(buf, sizeof(buf), "Activity:\nJUMPING");
          lv_obj_set_style_text_color(activity_label, lv_color_hex(0xFFDD00), 0);
          break;

        case ACT_MOVING:
          snprintf(buf, sizeof(buf), "Activity:\nMOVING");
          lv_obj_set_style_text_color(activity_label, lv_color_hex(0x00AAFF), 0);
          break;

        default:
          snprintf(buf, sizeof(buf), "Activity:\n--");
          lv_obj_set_style_text_color(activity_label, lv_color_hex(0x888888), 0);
          break;
      }
    }

    lv_label_set_text(activity_label, buf);
    lv_obj_invalidate(activity_label);
  }

  lv_refr_now(NULL);
}

// ═════════════════════════════════════════════════════════════════════════════
//  App helpers
// ═════════════════════════════════════════════════════════════════════════════
void forwardToApp(const String& payload) {
  if (pNotifyChar != nullptr && appConnected) {
    pNotifyChar->setValue(payload.c_str());
    pNotifyChar->notify();
    Serial.print("[APP NOTIFY] ");
    Serial.println(payload);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLE SERVER callbacks (ESP32 ↔ App)
// ═════════════════════════════════════════════════════════════════════════════
class AppServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pSrv) override {
    appConnected = true;
    Serial.println("[APP] App connected to ESP32 BLE server");
  }
  void onDisconnect(BLEServer* pSrv) override {
    appConnected = false;
    Serial.println("[APP] App disconnected — restarting advertising");
    BLEDevice::startAdvertising();
  }
};

class AppWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue();
    cmd.trim();
    Serial.print("[APP WRITE] Received: ");
    Serial.println(cmd);

    if (cmd == "PING") {
      Serial.println("[APP WRITE] PING acknowledged");
    } else if (cmd == "REQ_DATA") {
      forwardToApp(latestPayload);
    } else if (cmd == "STATUS") {
      String s = "PI:";
      s += (connected_status ? "1" : "0");
      s += ",APP:";
      s += (appConnected ? "1" : "0");
      forwardToApp(s);
    } else {
      Serial.println("[APP WRITE] Unknown command");
    }
  }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Start ESP32 BLE server
// ═════════════════════════════════════════════════════════════════════════════
void startESPServer() {
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new AppServerCallbacks());

  BLEService* service = pServer->createService(ESP_SERVICE_UUID);

  pNotifyChar = service->createCharacteristic(
    ESP_NOTIFY_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pNotifyChar->addDescriptor(new BLE2902());
  pNotifyChar->setValue("HR:0,SPO2:0,ACT:--,FALL:NORMAL");

  pWriteChar = service->createCharacteristic(
    ESP_WRITE_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );
  pWriteChar->setValue("READY");
  pWriteChar->setCallbacks(new AppWriteCallbacks());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(ESP_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[ESP SERVER] BLE server started — advertising as SEAM_ESP32");
}

// ═════════════════════════════════════════════════════════════════════════════
//  NOTIFY CALLBACK — data from PI-3
//  Parses: "HR,SPO2,ACTIVITY,FALL_STATE"
// ═════════════════════════════════════════════════════════════════════════════
static void notifyCallback(BLERemoteCharacteristic* chr,
                           uint8_t* data, size_t len, bool isNotify) {
  (void)chr; (void)isNotify;

  std::string s((const char*)data, (const char*)data + len);
  Serial.print("[PI RX] ");
  Serial.println(s.c_str());
  last_notify = millis();

  // Field 0: HR
  int c1 = s.find(',');
  if (c1 == (int)std::string::npos) { Serial.println("[PARSE] Missing field 1"); return; }
  int hr = atoi(s.substr(0, c1).c_str());
  if (hr >= 0 && hr <= 250) heartRate = hr;

  // Field 1: SpO2
  int c2 = s.find(',', c1 + 1);
  if (c2 == (int)std::string::npos) { Serial.println("[PARSE] Missing field 2"); return; }
  int spo2 = atoi(s.substr(c1 + 1, c2 - c1 - 1).c_str());
  if (spo2 >= 0 && spo2 <= 100) SpO2 = spo2;

  // Field 2: Activity
  int c3 = s.find(',', c2 + 1);
  if (c3 == (int)std::string::npos) { Serial.println("[PARSE] Missing field 3"); return; }
  std::string actStr = s.substr(c2 + 1, c3 - c2 - 1);

  if      (actStr == "STILL")   activityState = ACT_STILL;
  else if (actStr == "WALKING") activityState = ACT_WALKING;
  else if (actStr == "RUNNING") activityState = ACT_RUNNING;
  else if (actStr == "JUMPING") activityState = ACT_JUMPING;
  else if (actStr == "MOVING")  activityState = ACT_MOVING;
  else                          activityState = ACT_UNKNOWN;

  // Field 3: Fall state
  std::string fallStr = s.substr(c3 + 1);
  if      (fallStr == "FALL")      fallState = FALL_DETECTED;
  else if (fallStr == "SLIP_FALL") fallState = FALL_SLIP;
  else                             fallState = FALL_NORMAL;

  Serial.printf("  HR=%d  SpO2=%d  Act=%s  Fall=%s\n",
                heartRate, SpO2, actStr.c_str(), fallStr.c_str());

  // Build latestPayload for the app
  latestPayload  = "HR:"   + String(heartRate);
  latestPayload += ",SPO2:" + String(SpO2);
  latestPayload += ",ACT:"  + String(actStr.c_str());
  latestPayload += ",FALL:" + String(fallStr.c_str());

  update_UI = true;
  forwardToApp(latestPayload);
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLE CLIENT callbacks (ESP32 ↔ PI-3)
// ═════════════════════════════════════════════════════════════════════════════
class MyClientCallback : public BLEClientCallbacks {
  void onDisconnect(BLEClient*) override {
    Serial.println("[PI] Disconnected from PI-3");
    connected_status = false;
    update_UI        = true;
  }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Connect to PI-3
// ═════════════════════════════════════════════════════════════════════════════
bool connectToServer() {
  Serial.println("[PI] Connecting to PI-3…");

  if (pClient != nullptr) {
    if (pClient->isConnected()) pClient->disconnect();
    delete pClient;
    pClient = nullptr;
  }

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println("[PI] Connection failed");
    return false;
  }

  BLERemoteService* pSvc = pClient->getService(PI_SERVICE_UUID);
  if (!pSvc) {
    Serial.println("[PI] Service not found");
    pClient->disconnect();
    return false;
  }

  pRemoteChar = pSvc->getCharacteristic(PI_CHAR_UUID);
  if (!pRemoteChar) {
    Serial.println("[PI] Characteristic not found");
    pClient->disconnect();
    return false;
  }

  if (pRemoteChar->canNotify())
    pRemoteChar->registerForNotify(notifyCallback);

  Serial.println("[PI] Connected and subscribed to PI-3!");
  connected_status = true;
  update_UI        = true;
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLE Scan callback
// ═════════════════════════════════════════════════════════════════════════════
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(PI_SERVICE_UUID)) {
      Serial.println("[SCAN] Found PI-3!");
      BLEDevice::getScan()->stop();
      if (myDevice) { delete myDevice; myDevice = nullptr; }
      myDevice  = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

static void ble_start_scan() {
  Serial.println("[SCAN] Scanning for PI-3…");
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
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  SEAM ESP32 — Display + BLE Bridge V4   ");
  Serial.println("==========================================");

  lv_init();
  lv_xiao_disp_init();
  lv_xiao_touch_init();
  build_screen();

  BLEDevice::init("SEAM_ESP32");
  startESPServer();
  ble_start_scan();

  Serial.println("[SETUP] Ready.");
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  lv_timer_handler();
  delay(5);

  if (doConnect) {
    doConnect = false;
    if (!connectToServer()) {
      Serial.println("[MAIN] PI-3 connection failed — will rescan");
    }
  }

  if (pClient != nullptr && pClient->isConnected()) {
    if (last_notify != 0 && (millis() - last_notify) > Timeout) {
      last_notify      = 0;
      connected_status = false;
      Serial.println("[BLE] Notify timeout — disconnecting from PI-3");
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
      ble_start_scan();
    }
  }

  refresh_ui();
}
