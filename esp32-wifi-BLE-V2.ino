#include <lvgl.h>

#include <WiFi.h>
#include <WebServer.h>
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




// =========================
// Wi-Fi Access Point config
// =========================
const char* AP_SSID = "SEAM-ESP32";
const char* AP_PASS = "12345678";   // must be at least 8 chars

// =========================
// Web server
// =========================
WebServer server(80);

// =========================
// Shared sensor data
// =========================
float heartRateFloat = 0.0;
float spo2Float = 0.0;
float temperature = 0.0;
String piStatus = "Disconnected";
String esp32Status = "Broadcasting";
String lcdStatus = "Active";

unsigned long lastDataMillis = 0;
const unsigned long DATA_TIMEOUT_MS = 10000; // 10 sec timeout

// =========================
// Helper: JSON response
// =========================
String buildHealthJson() {
  String json = "{";
  json += "\"piStatus\":\"" + piStatus + "\",";
  json += "\"esp32Status\":\"" + esp32Status + "\",";
  json += "\"lcdStatus\":\"" + lcdStatus + "\"";
  json += "}";
  return json;
}

String buildDataJson() {
  String json = "{";
  json += "\"heartRate\":" + String(heartRateFloat, 1) + ",";
  json += "\"spo2\":" + String(spo2Float, 1) + ",";
  json += "\"temperature\":" + String(temperature, 1) + ",";
  json += "\"piStatus\":\"" + piStatus + "\",";
  json += "\"esp32Status\":\"" + esp32Status + "\",";
  json += "\"lcdStatus\":\"" + lcdStatus + "\"";
  json += "}";
  return json;
}

// =========================
// Routes
// =========================
void handleRoot() {
  String msg = "";
  msg += "SEAM ESP32 Bridge Running\n";
  msg += "Open /health or /data\n";
  msg += "AP IP: " + WiFi.softAPIP().toString() + "\n";
  server.send(200, "text/plain", msg);
}

void handleHealth() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buildHealthJson());
}

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buildDataJson());
}

void handleNotFound() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "application/json", "{\"error\":\"Not found\"}");
}
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
static BLEClient*               pClient         = nullptr;
static BLERemoteCharacteristic* pRemoteChar     = nullptr;
static BLEAdvertisedDevice*     myDevice        = nullptr;
static bool                     doConnect       = false;
static uint32_t                 last_scan       = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  BLE handles — SERVER (ESP32 → App)
// ═════════════════════════════════════════════════════════════════════════════
static BLEServer*         pServer      = nullptr;
static BLECharacteristic* pNotifyChar  = nullptr;
static BLECharacteristic* pWriteChar   = nullptr;
static bool               appConnected = false;

// ═════════════════════════════════════════════════════════════════════════════
//  Parsed vitals
// ═════════════════════════════════════════════════════════════════════════════
volatile int heartRate = 0;
volatile int SpO2      = 0;

enum ActivityState {
  ACT_STILL, ACT_WALKING, ACT_RUNNING, ACT_JUMPING, ACT_MOVING, ACT_UNKNOWN
};

enum FallState {
  FALL_NORMAL, FALL_DETECTED, FALL_SLIP
};

volatile ActivityState activityState = ACT_UNKNOWN;
volatile FallState     fallState     = FALL_NORMAL;
String latestPayload = "HR:0,SPO2:0,ACT:--,FALL:NORMAL";

static bool     connected_status = false;
static uint32_t last_notify      = 0;
static const uint32_t Timeout    = 10000;
volatile bool   update_UI        = false;

// ═════════════════════════════════════════════════════════════════════════════
//  Alert / UI flow state
// ═════════════════════════════════════════════════════════════════════════════
#define ALERT_COUNTDOWN_SEC      10
#define ALERT_SUPPRESS_MS        3000
#define RESPONSE_SHOW_MS         1500
#define EMERGENCY_SCREEN_MS      3500
#define SCREEN_TRANSITION_MS      280

enum UiMode {
  UI_MAIN,
  UI_ALERT,
  UI_RESPONSE,
  UI_EMERGENCY
};

static UiMode      uiMode              = UI_MAIN;
static FallState   alertFallType       = FALL_NORMAL;
static uint32_t    alertStartMs        = 0;
static int         alertLastSecShown   = -1;
static uint32_t    uiModeStartMs       = 0;

static volatile bool pendingAlertRequest = false;
static volatile int  pendingAlertType    = (int)FALL_NORMAL;
static bool          remoteFallLatched   = false;
static uint32_t      alertSuppressUntil  = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  LVGL objects
// ═════════════════════════════════════════════════════════════════════════════
static lv_obj_t* main_screen       = nullptr;
static lv_obj_t* hr_label          = nullptr;
static lv_obj_t* activity_label    = nullptr;
static lv_obj_t* status_dot        = nullptr;

static lv_obj_t* alert_screen      = nullptr;
static lv_obj_t* alert_title_lbl   = nullptr;
static lv_obj_t* alert_sub_lbl     = nullptr;
static lv_obj_t* alert_count_lbl   = nullptr;
static lv_obj_t* alert_yes_btn     = nullptr;
static lv_obj_t* alert_no_btn      = nullptr;

static lv_obj_t* response_screen   = nullptr;
static lv_obj_t* response_title    = nullptr;
static lv_obj_t* response_sub      = nullptr;

static lv_obj_t* emergency_screen  = nullptr;
static lv_obj_t* emergency_title   = nullptr;
static lv_obj_t* emergency_sub     = nullptr;
static lv_obj_t* emergency_spinner = nullptr;

// ═════════════════════════════════════════════════════════════════════════════
//  Alert history
// ═════════════════════════════════════════════════════════════════════════════
#define FALL_LOG_MAX 20
struct FallLogEntry {
  uint32_t      timestamp_ms;
  FallState     type;
  ActivityState activity;
  int           hr;
  int           spo2;
};
static FallLogEntry fallLog[FALL_LOG_MAX];
static uint8_t      fallLogHead  = 0;
static uint8_t      fallLogCount = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  Helpers
// ═════════════════════════════════════════════════════════════════════════════
static const char* activityStr(ActivityState a) {
  switch (a) {
    case ACT_STILL:   return "STILL";
    case ACT_WALKING: return "WALKING";
    case ACT_RUNNING: return "RUNNING";
    case ACT_JUMPING: return "JUMPING";
    case ACT_MOVING:  return "MOVING";
    default:          return "UNKNOWN";
  }
}

static const char* fallTypeStr(FallState f) {
  switch (f) {
    case FALL_DETECTED: return "FALL";
    case FALL_SLIP:     return "SLIP_FALL";
    default:            return "NORMAL";
  }
}

static void forwardToApp(const String& payload) {
  if (pNotifyChar != nullptr && appConnected) {
    pNotifyChar->setValue(payload.c_str());
    pNotifyChar->notify();
    Serial.print("[APP NOTIFY] ");
    Serial.println(payload);
  }
}

static void logFallEvent(FallState type, ActivityState act, int hr, int spo2) {
  fallLog[fallLogHead] = { millis(), type, act, hr, spo2 };
  fallLogHead = (fallLogHead + 1) % FALL_LOG_MAX;
  if (fallLogCount < FALL_LOG_MAX) fallLogCount++;
}

static void sendLogToApp(uint8_t count) {
  if (!appConnected || pNotifyChar == nullptr) return;
  if (fallLogCount == 0) {
    forwardToApp("LOG_EMPTY");
    return;
  }
  if (count == 0 || count > fallLogCount) count = fallLogCount;

  uint32_t now      = millis();
  uint8_t  startIdx = (fallLogHead - count + FALL_LOG_MAX) % FALL_LOG_MAX;
  char buf[96];

  for (uint8_t i = 0; i < count; i++) {
    uint8_t idx = (startIdx + i) % FALL_LOG_MAX;
    const FallLogEntry& e = fallLog[idx];
    snprintf(buf, sizeof(buf), "LOG,%u,%lu,%s,%s,%d,%d",
             (unsigned)(i + 1),
             (unsigned long)((now - e.timestamp_ms) / 1000UL),
             fallTypeStr(e.type),
             activityStr(e.activity),
             e.hr, e.spo2);
    pNotifyChar->setValue(buf);
    pNotifyChar->notify();
    delay(20);
  }
  forwardToApp("LOG_END");
}

static void clearLog() {
  fallLogHead = 0;
  fallLogCount = 0;
  forwardToApp("LOG_CLEARED");
}

static lv_color_t activityColor(ActivityState a) {
  switch (a) {
    case ACT_STILL:   return lv_color_hex(0x00CC66);
    case ACT_WALKING: return lv_color_hex(0x00DDFF);
    case ACT_RUNNING: return lv_color_hex(0xFF8800);
    case ACT_JUMPING: return lv_color_hex(0xFFDD00);
    case ACT_MOVING:  return lv_color_hex(0x00AAFF);
    default:          return lv_color_hex(0x888888);
  }
}

static lv_color_t alertColor(FallState f) {
  return (f == FALL_SLIP) ? lv_color_hex(0xFFA500) : lv_color_hex(0xFF3B1F);
}

static lv_color_t alertColorDim(FallState f) {
  return (f == FALL_SLIP) ? lv_color_hex(0xCC8400) : lv_color_hex(0xCC2A14);
}

static void queueAlertRequest(FallState ftype) {
  pendingAlertType = (int)ftype;
  pendingAlertRequest = true;
}

static void loadScreenAnimated(lv_obj_t* scr) {
  lv_scr_load_anim(scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, SCREEN_TRANSITION_MS, 0, false);
}

// ═════════════════════════════════════════════════════════════════════════════
//  UI event callbacks
// ═════════════════════════════════════════════════════════════════════════════
static void showResponseScreen(bool userOkay);
static void showEmergencyScreen();
static void triggerEmergency();
static void dismissAlert(bool userOkay);

static void cb_yes(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    Serial.println("[ALERT] YES pressed");
    dismissAlert(true);
  }
}

static void cb_no(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    Serial.println("[ALERT] NO pressed");
    dismissAlert(false);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  Build main screen
// ═════════════════════════════════════════════════════════════════════════════
static void build_main_screen() {
  main_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(main_screen, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(main_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(main_screen, 0, 0);
  lv_obj_set_style_pad_all(main_screen, 0, 0);

  status_dot = lv_obj_create(main_screen);
  lv_obj_set_size(status_dot, 12, 12);
  lv_obj_align(status_dot, LV_ALIGN_TOP_RIGHT, -16, 14);
  lv_obj_set_style_radius(status_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(status_dot, lv_color_hex(0x666666), 0);
  lv_obj_set_style_border_width(status_dot, 0, 0);

  hr_label = lv_label_create(main_screen);
  lv_label_set_text(hr_label, "HR: -- bpm\nSpO2: --%");
  lv_obj_set_style_text_align(hr_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(hr_label, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(hr_label, &lv_font_montserrat_14, 0);
  lv_obj_align(hr_label, LV_ALIGN_CENTER, 0, -42);

  lv_obj_t* divider = lv_obj_create(main_screen);
  lv_obj_set_size(divider, 170, 2);
  lv_obj_align(divider, LV_ALIGN_CENTER, 0, 2);
  lv_obj_set_style_bg_color(divider, lv_color_hex(0x444444), 0);
  lv_obj_set_style_border_width(divider, 0, 0);

  activity_label = lv_label_create(main_screen);
  lv_label_set_text(activity_label, "Activity\n--");
  lv_obj_set_style_text_align(activity_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(activity_label, lv_color_hex(0x888888), 0);
  lv_obj_set_style_text_font(activity_label, &lv_font_montserrat_14, 0);
  lv_obj_align(activity_label, LV_ALIGN_CENTER, 0, 48);

  lv_scr_load(main_screen);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Build alert screen
// ═════════════════════════════════════════════════════════════════════════════
static void build_alert_screen() {
  alert_screen = lv_obj_create(NULL);
  lv_obj_set_size(alert_screen, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_style_bg_color(alert_screen, lv_color_hex(0xFF3B1F), 0);
  lv_obj_set_style_bg_opa(alert_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(alert_screen, 0, 0);
  lv_obj_set_style_pad_all(alert_screen, 0, 0);

  alert_title_lbl = lv_label_create(alert_screen);
  lv_label_set_text(alert_title_lbl, "FALL\nDETECTED");
  lv_obj_set_style_text_color(alert_title_lbl, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(alert_title_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(alert_title_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(alert_title_lbl, LV_ALIGN_TOP_MID, 0, 16);

  alert_sub_lbl = lv_label_create(alert_screen);
  lv_label_set_text(alert_sub_lbl, "Tap screen to respond");
  lv_obj_set_style_text_color(alert_sub_lbl, lv_color_hex(0xFFF2D8), 0);
  lv_obj_set_style_text_align(alert_sub_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(alert_sub_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(alert_sub_lbl, LV_ALIGN_TOP_MID, 0, 74);

  alert_count_lbl = lv_label_create(alert_screen);
  lv_label_set_text(alert_count_lbl, "10");
  lv_obj_set_style_text_color(alert_count_lbl, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(alert_count_lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(alert_count_lbl, &lv_font_montserrat_14, 0);
  lv_obj_align(alert_count_lbl, LV_ALIGN_CENTER, 0, -6);

  alert_yes_btn = lv_btn_create(alert_screen);
  lv_obj_set_size(alert_yes_btn, 92, 68);
  lv_obj_align(alert_yes_btn, LV_ALIGN_BOTTOM_LEFT, 16, -18);
  lv_obj_set_style_bg_color(alert_yes_btn, lv_color_hex(0x00C853), 0);
  lv_obj_set_style_bg_color(alert_yes_btn, lv_color_hex(0x009B3A), LV_STATE_PRESSED);
  lv_obj_set_style_bg_opa(alert_yes_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(alert_yes_btn, 22, 0);
  lv_obj_set_style_border_width(alert_yes_btn, 0, 0);
  lv_obj_add_event_cb(alert_yes_btn, cb_yes, LV_EVENT_CLICKED, NULL);

  lv_obj_t* yes_lbl = lv_label_create(alert_yes_btn);
  lv_label_set_text(yes_lbl, "YES");
  lv_obj_set_style_text_color(yes_lbl, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(yes_lbl, &lv_font_montserrat_14, 0);
  lv_obj_center(yes_lbl);

  alert_no_btn = lv_btn_create(alert_screen);
  lv_obj_set_size(alert_no_btn, 92, 68);
  lv_obj_align(alert_no_btn, LV_ALIGN_BOTTOM_RIGHT, -16, -18);
  lv_obj_set_style_bg_color(alert_no_btn, lv_color_hex(0xD50000), 0);
  lv_obj_set_style_bg_color(alert_no_btn, lv_color_hex(0x9A0000), LV_STATE_PRESSED);
  lv_obj_set_style_bg_opa(alert_no_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(alert_no_btn, 22, 0);
  lv_obj_set_style_border_width(alert_no_btn, 0, 0);
  lv_obj_add_event_cb(alert_no_btn, cb_no, LV_EVENT_CLICKED, NULL);

  lv_obj_t* no_lbl = lv_label_create(alert_no_btn);
  lv_label_set_text(no_lbl, "NO");
  lv_obj_set_style_text_color(no_lbl, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(no_lbl, &lv_font_montserrat_14, 0);
  lv_obj_center(no_lbl);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Build response screen
// ═════════════════════════════════════════════════════════════════════════════
static void build_response_screen() {
  response_screen = lv_obj_create(NULL);
  lv_obj_set_size(response_screen, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_style_bg_color(response_screen, lv_color_hex(0x102418), 0);
  lv_obj_set_style_bg_opa(response_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(response_screen, 0, 0);
  lv_obj_set_style_pad_all(response_screen, 0, 0);

  response_title = lv_label_create(response_screen);
  lv_label_set_text(response_title, "OK");
  lv_obj_set_style_text_color(response_title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(response_title, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(response_title, &lv_font_montserrat_14, 0);
  lv_obj_align(response_title, LV_ALIGN_CENTER, 0, -18);

  response_sub = lv_label_create(response_screen);
  lv_label_set_text(response_sub, "Continuing monitoring");
  lv_obj_set_style_text_color(response_sub, lv_color_hex(0xCCFFDD), 0);
  lv_obj_set_style_text_align(response_sub, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(response_sub, &lv_font_montserrat_14, 0);
  lv_obj_align(response_sub, LV_ALIGN_CENTER, 0, 28);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Build emergency screen
// ═════════════════════════════════════════════════════════════════════════════
static void build_emergency_screen() {
  emergency_screen = lv_obj_create(NULL);
  lv_obj_set_size(emergency_screen, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_style_bg_color(emergency_screen, lv_color_hex(0x240C0C), 0);
  lv_obj_set_style_bg_opa(emergency_screen, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(emergency_screen, 0, 0);
  lv_obj_set_style_pad_all(emergency_screen, 0, 0);

  emergency_title = lv_label_create(emergency_screen);
  lv_label_set_text(emergency_title, "Calling\nEmergency");
  lv_obj_set_style_text_color(emergency_title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(emergency_title, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(emergency_title, &lv_font_montserrat_14, 0);
  lv_obj_align(emergency_title, LV_ALIGN_TOP_MID, 0, 28);

  emergency_spinner = lv_spinner_create(emergency_screen);
  lv_obj_set_size(emergency_spinner, 54, 54);
  lv_obj_align(emergency_spinner, LV_ALIGN_CENTER, 0, 12);

  emergency_sub = lv_label_create(emergency_screen);
  lv_label_set_text(emergency_sub, "Sending alert...\nPlease stay still");
  lv_obj_set_style_text_color(emergency_sub, lv_color_hex(0xFFDDDD), 0);
  lv_obj_set_style_text_align(emergency_sub, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_font(emergency_sub, &lv_font_montserrat_14, 0);
  lv_obj_align(emergency_sub, LV_ALIGN_BOTTOM_MID, 0, -22);
}

// ═════════════════════════════════════════════════════════════════════════════
//  UI state changes
// ═════════════════════════════════════════════════════════════════════════════
static void showAlertScreen(FallState ftype) {
  alertFallType = ftype;
  alertStartMs = millis();
  alertLastSecShown = -1;
  uiMode = UI_ALERT;
  uiModeStartMs = millis();

  bool isSlip = (ftype == FALL_SLIP);
  lv_obj_set_style_bg_color(alert_screen, alertColor(ftype), 0);
  lv_label_set_text(alert_title_lbl, isSlip ? "SLIP / FALL" : "FALL\nDETECTED");
  lv_label_set_text(alert_sub_lbl, "Tap screen to respond");
  lv_label_set_text(alert_count_lbl, String(ALERT_COUNTDOWN_SEC).c_str());

  loadScreenAnimated(alert_screen);
  forwardToApp(isSlip ? "ALERT:SLIP_FALL" : "ALERT:FALL");
}

static void showResponseScreen(bool userOkay) {
  uiMode = UI_RESPONSE;
  uiModeStartMs = millis();

  lv_obj_set_style_bg_color(response_screen,
                            userOkay ? lv_color_hex(0x102418) : lv_color_hex(0x2A1212), 0);
  lv_label_set_text(response_title, userOkay ? "OK" : "Alert Sent");
  lv_label_set_text(response_sub,
                    userOkay ? "Continuing monitoring" : "Emergency contact notified");

  loadScreenAnimated(response_screen);
}

static void showEmergencyScreen() {
  uiMode = UI_EMERGENCY;
  uiModeStartMs = millis();
  loadScreenAnimated(emergency_screen);
}

static void triggerEmergency() {
  forwardToApp("EMERGENCY");
  showEmergencyScreen();
}

static void dismissAlert(bool userOkay) {
  fallState = FALL_NORMAL;
  last_notify = millis();
  alertSuppressUntil = millis() + ALERT_SUPPRESS_MS;
  remoteFallLatched = true;

  if (userOkay) {
    forwardToApp("USER_OK");
    showResponseScreen(true);
  } else {
    triggerEmergency();
  }
}

static void tickAlertCountdown() {
  uint32_t elapsed = millis() - alertStartMs;
  int secLeft = ALERT_COUNTDOWN_SEC - (int)(elapsed / 1000);

  if (secLeft <= 0) {
    Serial.println("[ALERT] Countdown expired");
    dismissAlert(false);
    return;
  }

  if (secLeft != alertLastSecShown) {
    alertLastSecShown = secLeft;
    lv_label_set_text(alert_count_lbl, String(secLeft).c_str());
    lv_obj_set_style_bg_color(alert_screen,
                              (secLeft % 2 == 0) ? alertColor(alertFallType)
                                                : alertColorDim(alertFallType),
                              0);
  }
}

static void tickUiScreens() {
  if (uiMode == UI_RESPONSE) {
    if ((millis() - uiModeStartMs) >= RESPONSE_SHOW_MS) {
      uiMode = UI_MAIN;
      loadScreenAnimated(main_screen);
      update_UI = true;
    }
  } else if (uiMode == UI_EMERGENCY) {
    if ((millis() - uiModeStartMs) >= EMERGENCY_SCREEN_MS) {
      showResponseScreen(false);
    }
  }
}

static void refresh_ui() {
  if (!update_UI) return;
  update_UI = false;

  if (hr_label) {
    char buf[48];
    if (connected_status) {
      snprintf(buf, sizeof(buf), "HR: %d bpm\nSpO2: %d%%", heartRate, SpO2);
      lv_obj_set_style_bg_color(status_dot, lv_color_hex(0x00C853), 0);
    } else {
      snprintf(buf, sizeof(buf), "HR: -- bpm\nSpO2: --%%");
      lv_obj_set_style_bg_color(status_dot, lv_color_hex(0x666666), 0);
    }
    lv_label_set_text(hr_label, buf);
  }

  if (activity_label) {
    char buf[64];
    if (!connected_status) {
      snprintf(buf, sizeof(buf), "Activity\n--");
      lv_obj_set_style_text_color(activity_label, lv_color_hex(0x888888), 0);
    } else if (fallState == FALL_DETECTED) {
      snprintf(buf, sizeof(buf), "FALL\nDETECTED");
      lv_obj_set_style_text_color(activity_label, lv_color_hex(0xFF3B1F), 0);
    } else if (fallState == FALL_SLIP) {
      snprintf(buf, sizeof(buf), "SLIP / FALL");
      lv_obj_set_style_text_color(activity_label, lv_color_hex(0xFFA500), 0);
    } else {
      snprintf(buf, sizeof(buf), "Activity\n%s", activityStr(activityState));
      lv_obj_set_style_text_color(activity_label, activityColor(activityState), 0);
    }
    lv_label_set_text(activity_label, buf);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLE server callbacks
// ═════════════════════════════════════════════════════════════════════════════
class AppServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    appConnected = true;
    Serial.println("[APP] Connected");
  }
  void onDisconnect(BLEServer*) override {
    appConnected = false;
    Serial.println("[APP] Disconnected");
    BLEDevice::startAdvertising();
  }
};

class AppWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String cmd = pChar->getValue().c_str();
    cmd.trim();
    Serial.print("[APP WRITE] ");
    Serial.println(cmd);

    if (cmd == "PING") {
      return;
    } else if (cmd == "REQ_DATA") {
      forwardToApp(latestPayload);
    } else if (cmd == "STATUS") {
      String s = "PI:" + String(connected_status ? 1 : 0) + ",APP:" +
                 String(appConnected ? 1 : 0) + ",LOG:" + String(fallLogCount);
      forwardToApp(s);
    } else if (cmd == "GET_LOG" || cmd.startsWith("GET_LOG:")) {
      uint8_t requested = 0;
      if (cmd.startsWith("GET_LOG:")) {
        String numStr = cmd.substring(8);
        numStr.trim();
        int n = numStr.toInt();
        if (n > 0 && n <= FALL_LOG_MAX) requested = (uint8_t)n;
      }
      sendLogToApp(requested);
    } else if (cmd == "CLEAR_LOG") {
      clearLog();
    }
  }
};

static void startESPServer() {
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
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
}

void updateSensorData(float hr, float ox, float temp) {
  heartRateFloat = hr;
  spo2Float = ox;
  temperature = temp;
  piStatus = "Connected";
  lastDataMillis = millis();

  // TODO:
  // also update your LCD here if needed
}

// ═════════════════════════════════════════════════════════════════════════════
//  PI notify callback
// ═════════════════════════════════════════════════════════════════════════════
static void notifyCallback(BLERemoteCharacteristic* chr,
                           uint8_t* data, size_t len, bool isNotify) {
  (void)chr;
  (void)isNotify;

  std::string s((const char*)data, (const char*)data + len);
  last_notify = millis();
  Serial.print("[PI RX] ");
  Serial.println(s.c_str());

  int c1 = s.find(',');
  if (c1 == (int)std::string::npos) return;
  int c2 = s.find(',', c1 + 1);
  if (c2 == (int)std::string::npos) return;
  int c3 = s.find(',', c2 + 1);
  if (c3 == (int)std::string::npos) return;

  int hr = atoi(s.substr(0, c1).c_str());
  int parsedSpo2 = atoi(s.substr(c1 + 1, c2 - c1 - 1).c_str());
  std::string actStr = s.substr(c2 + 1, c3 - c2 - 1);
  std::string incomingFallStr = s.substr(c3 + 1);

  if (hr >= 0 && hr <= 250) heartRate = hr;
  if (parsedSpo2 >= 0 && parsedSpo2 <= 100) SpO2 = parsedSpo2;

  if      (actStr == "STILL")   activityState = ACT_STILL;
  else if (actStr == "WALKING") activityState = ACT_WALKING;
  else if (actStr == "RUNNING") activityState = ACT_RUNNING;
  else if (actStr == "JUMPING") activityState = ACT_JUMPING;
  else if (actStr == "MOVING")  activityState = ACT_MOVING;
  else                          activityState = ACT_UNKNOWN;

  FallState incomingFall = FALL_NORMAL;
  if      (incomingFallStr == "FALL")      incomingFall = FALL_DETECTED;
  else if (incomingFallStr == "SLIP_FALL") incomingFall = FALL_SLIP;

  if (incomingFall == FALL_NORMAL) {
    remoteFallLatched = false;
  }

  fallState = incomingFall;

  if (incomingFall != FALL_NORMAL &&
      !remoteFallLatched &&
      uiMode == UI_MAIN &&
      millis() >= alertSuppressUntil) {
    remoteFallLatched = true;
    logFallEvent(incomingFall, activityState, heartRate, SpO2);
    queueAlertRequest(incomingFall);
  }

  latestPayload  = "HR:"    + String(heartRate);
  latestPayload += ",SPO2:" + String(SpO2);
  latestPayload += ",ACT:"  + String(actStr.c_str());
  latestPayload += ",FALL:" + String(incomingFallStr.c_str());

  update_UI = true;
  updateSensorData(heartRate, SpO2, 97);
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLE client callbacks / connect
// ═════════════════════════════════════════════════════════════════════════════
class MyClientCallback : public BLEClientCallbacks {
  void onDisconnect(BLEClient*) override {
    connected_status = false;
    update_UI = true;
    Serial.println("[PI] Disconnected from PI-3");
  }
};

static bool connectToServer() {
  Serial.println("[PI] Connecting...");

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
    pClient->disconnect();
    return false;
  }

  pRemoteChar = pSvc->getCharacteristic(PI_CHAR_UUID);
  if (!pRemoteChar) {
    pClient->disconnect();
    return false;
  }

  if (pRemoteChar->canNotify()) pRemoteChar->registerForNotify(notifyCallback);

  connected_status = true;
  update_UI = true;
  Serial.println("[PI] Connected and subscribed");
  return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(PI_SERVICE_UUID)) {
      BLEDevice::getScan()->stop();
      if (myDevice) {
        delete myDevice;
        myDevice = nullptr;
      }
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      Serial.println("[SCAN] Found PI-3");
    }
  }
};

// =========================
// Scan callback (static to avoid heap leak on repeated scans)
// Must be declared before ble_start_scan()
// =========================
static MyAdvertisedDeviceCallbacks scanCallbacks;

static void ble_start_scan() {
  BLEDevice::getScan()->clearResults();
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(&scanCallbacks);
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);
  pScan->start(5, true);
  last_scan = millis();
  Serial.println("[SCAN] Scanning for PI-3...");
}

// =========================
// Demo data generator
// Remove after BLE is working
// =========================
void updateDemoData() {
  // TODO: replace with real BLE data
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Starting SEAM ESP32 Bridge...");

  // Start Wi-Fi AP
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (!apOk) {
    Serial.println("Failed to start AP!");
  } else {
    Serial.println("AP started successfully.");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASS);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/health", HTTP_GET, handleHealth);
  server.on("/data", HTTP_GET, handleData);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web server started.");

  // TODO:
  // Initialize LCD here
  // Initialize BLE here
  // Start BLE client / receiver here

  lv_init();
  lv_xiao_disp_init();
  lv_xiao_touch_init();

  build_main_screen();
  build_alert_screen();
  build_response_screen();
  build_emergency_screen();

  BLEDevice::init("SEAM_ESP32");
  startESPServer();
  ble_start_scan();

  update_UI = true;
  Serial.println("[SETUP] Ready");
}

// =========================
// Loop
// =========================
void loop() {
  server.handleClient();

  // TEMP: demo values so app works immediately
  updateDemoData();

  // If no new data for a while, mark Pi disconnected
  if (millis() - lastDataMillis > DATA_TIMEOUT_MS) {
    piStatus = "Disconnected";
  }

  // TODO:
  // Process BLE here
  // When BLE payload arrives, call:
  // updateSensorData(receivedHR, receivedSpO2, receivedTemp);
  lv_timer_handler();
  delay(5);

  if (pendingAlertRequest && uiMode == UI_MAIN) {
    pendingAlertRequest = false;
    showAlertScreen((FallState)pendingAlertType);
  }

  if (uiMode == UI_ALERT) {
    tickAlertCountdown();
  } else {
    tickUiScreens();
  }

  if (doConnect) {
    doConnect = false;
    if (!connectToServer()) {
      Serial.println("[MAIN] PI-3 connect failed, rescanning");
    }
  }

  if (pClient != nullptr && pClient->isConnected()) {
    if (last_notify != 0 && (millis() - last_notify) > Timeout) {
      last_notify = 0;
      connected_status = false;
      Serial.println("[BLE] Notify timeout, disconnecting");
      pClient->disconnect();
      update_UI = true;
    } else {
      connected_status = true;
    }
  } else if (!doConnect) {
    if (connected_status) {
      connected_status = false;
      update_UI = true;
    }
    if (millis() - last_scan > 6000) {
      ble_start_scan();
    }
  }

  refresh_ui();
}