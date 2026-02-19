// ------------------ Graphics backend selection ------------------
// Pick ONE backend.
// For your setup (you have GFX_Library_for_Arduino installed), use Arduino_GFX:
#define USE_ARDUINO_GFX_LIBRARY
// #define USE_TFT_ESPI_LIBRARY   // <- leave this OFF unless you're using TFT_eSPI

// Fix for Seeed header using BLACK
#ifndef BLACK
#define BLACK 0x0000
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

#include <lvgl.h>
#include "lv_xiao_round_screen.h"

// BLE (ESP32 Arduino core)
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>

// ------------------- USER SETTINGS -------------------
static const char* WIFI_SSID = "YOUR_WIFI_SSID";
static const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// Pi Zero should advertise this service, and notify on this characteristic.
static const char* SVC_UUID = "6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10";
static const char* CHR_UUID = "c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11";

// Optional: NTP config (for real time instead of uptime)
static const char* NTP_SERVER = "pool.ntp.org";
static const long  GMT_OFFSET_SEC = -5 * 3600;     // America/New_York (standard)
static const int   DAYLIGHT_OFFSET_SEC = 3600;     // DST
// -----------------------------------------------------

// ------------------- UI state -------------------
enum ScreenMode { SCREEN_HOME = 0, SCREEN_HR = 1, SCREEN_SPO2 = 2, SCREEN_STATUS = 3 };
static ScreenMode g_screen = SCREEN_HOME;

// LVGL objects
static lv_obj_t* g_label_title  = nullptr;
static lv_obj_t* g_label_time   = nullptr;
static lv_obj_t* g_label_status = nullptr;

static lv_obj_t* g_chart_hr   = nullptr;
static lv_obj_t* g_chart_spo2 = nullptr;
static lv_chart_series_t* g_s_hr   = nullptr;
static lv_chart_series_t* g_s_spo2 = nullptr;

static const int CHART_POINTS = 80;

// Latest values
static volatile int g_hr   = 0;
static volatile int g_spo2 = 0;

// BLE client refs/state
static BLEScan* g_scan = nullptr;
static BLEClient* g_client = nullptr;
static BLERemoteCharacteristic* g_remoteChr = nullptr;
static bool g_ble_connected = false;
static bool g_ble_scanning  = false;

// Timing for LVGL ticks
static uint32_t g_last_lv_tick = 0;

// ------------------- Helpers: WiFi + Time -------------------
static void wifi_start()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 6000) {
    delay(150);
  }

  if (WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  }
}

static bool get_time_string(char* out, size_t n)
{
  // Try real time first (NTP)
  struct tm timeinfo;
  if (WiFi.status() == WL_CONNECTED && getLocalTime(&timeinfo, 50)) {
    // HH:MM:SS
    strftime(out, n, "%H:%M:%S", &timeinfo);
    return true;
  }

  // Fallback: uptime MM:SS
  uint32_t s = millis() / 1000;
  uint32_t mm = (s / 60) % 100;
  uint32_t ss = s % 60;
  snprintf(out, n, "%02lu:%02lu", (unsigned long)mm, (unsigned long)ss);
  return false;
}

static const char* wifi_state_str()
{
  return (WiFi.status() == WL_CONNECTED) ? "WiFi: connected" : "WiFi: not connected";
}

static const char* ble_state_str()
{
  if (g_ble_connected) return "BLE: connected";
  if (g_ble_scanning)  return "BLE: scanning";
  return "BLE: idle";
}

// ------------------- BLE: parse incoming data -------------------
static void parse_payload(const std::string& s)
{
  // Accept: "78,97"
  // Or: "HR=78,SPO2=97"
  int hr = -1, spo2 = -1;

  auto comma = s.find(',');
  if (comma != std::string::npos) {
    // Try CSV numbers
    try {
      hr = atoi(s.substr(0, comma).c_str());
      spo2 = atoi(s.substr(comma + 1).c_str());
    } catch (...) {}
  }

  if (hr < 0 || spo2 < 0) {
    // Try labeled
    auto hrPos = s.find("HR=");
    auto spPos = s.find("SPO2=");
    if (hrPos != std::string::npos) hr = atoi(s.substr(hrPos + 3).c_str());
    if (spPos != std::string::npos) spo2 = atoi(s.substr(spPos + 5).c_str());
  }

  if (hr >= 0 && hr <= 250)   g_hr = hr;
  if (spo2 >= 0 && spo2 <= 100) g_spo2 = spo2;
}

static void notify_cb(BLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify)
{
  (void)chr; (void)isNotify;
  std::string s((const char*)data, (const char*)data + len);
  parse_payload(s);
}

// ------------------- BLE: scan/connect logic -------------------
class AdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveServiceUUID()) return;

    BLEUUID target(SVC_UUID);
    if (!advertisedDevice.isAdvertisingService(target)) return;

    // Found the Pi peripheral
    g_ble_scanning = false;
    g_scan->stop();

    // Connect
    g_client = BLEDevice::createClient();
    if (!g_client->connect(&advertisedDevice)) {
      g_ble_connected = false;
      return;
    }

    BLERemoteService* svc = g_client->getService(BLEUUID(SVC_UUID));
    if (!svc) {
      g_client->disconnect();
      g_ble_connected = false;
      return;
    }

    g_remoteChr = svc->getCharacteristic(BLEUUID(CHR_UUID));
    if (!g_remoteChr) {
      g_client->disconnect();
      g_ble_connected = false;
      return;
    }

    // Subscribe to notify if supported
    if (g_remoteChr->canNotify()) {
      g_remoteChr->registerForNotify(notify_cb);

      // Some stacks want CCCD; harmless if not needed
      BLERemoteDescriptor* cccd = g_remoteChr->getDescriptor(BLEUUID((uint16_t)0x2902));
      if (cccd) {
        uint8_t notifyOn[2] = {0x01, 0x00};
        cccd->writeValue(notifyOn, 2, true);
      }
    }

    g_ble_connected = true;
  }
};

static void ble_start_scan()
{
  BLEDevice::init("XIAO-Round-Display");

  g_scan = BLEDevice::getScan();
  g_scan->setAdvertisedDeviceCallbacks(new AdvCB(), true);
  g_scan->setInterval(1349);
  g_scan->setWindow(449);
  g_scan->setActiveScan(true);

  g_ble_connected = false;
  g_ble_scanning = true;

  // Non-blocking-ish scan (short burst). We'll restart periodically if needed.
  g_scan->start(3, false);
}

// ------------------- UI build -------------------
static void install_tap_handler(); // forward

static void build_home_screen()
{
  lv_obj_clean(lv_scr_act());

  g_label_title = lv_label_create(lv_scr_act());
  lv_label_set_text(g_label_title, "Hello World");
  lv_obj_align(g_label_title, LV_ALIGN_TOP_MID, 0, 18);

  g_label_time = lv_label_create(lv_scr_act());
  lv_label_set_text(g_label_time, "--:--");
  lv_obj_align(g_label_time, LV_ALIGN_TOP_MID, 0, 48);

  lv_obj_t* hint = lv_label_create(lv_scr_act());
  lv_label_set_text(hint, "Tap: HR -> SpO2 -> Status");
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -18);

  install_tap_handler();
}

static void build_hr_screen()
{
  lv_obj_clean(lv_scr_act());

  lv_obj_t* title = lv_label_create(lv_scr_act());
  lv_label_set_text(title, "Heart Rate");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

  // Smaller chart
  g_chart_hr = lv_chart_create(lv_scr_act());
  lv_obj_set_size(g_chart_hr, 210, 135);
  lv_obj_align(g_chart_hr, LV_ALIGN_CENTER, 0, 10);

  lv_chart_set_type(g_chart_hr, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(g_chart_hr, CHART_POINTS);
  lv_chart_set_range(g_chart_hr, LV_CHART_AXIS_PRIMARY_Y, 40, 180);

  g_s_hr = lv_chart_add_series(g_chart_hr, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

  for (int i = 0; i < CHART_POINTS; i++) lv_chart_set_next_value(g_chart_hr, g_s_hr, 0);

  lv_obj_t* hint = lv_label_create(lv_scr_act());
  lv_label_set_text(hint, "Tap to go to SpO2");
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);

  install_tap_handler();
}

static void build_spo2_screen()
{
  lv_obj_clean(lv_scr_act());

  lv_obj_t* title = lv_label_create(lv_scr_act());
  lv_label_set_text(title, "SpO2");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

  g_chart_spo2 = lv_chart_create(lv_scr_act());
  lv_obj_set_size(g_chart_spo2, 210, 135);
  lv_obj_align(g_chart_spo2, LV_ALIGN_CENTER, 0, 10);

  lv_chart_set_type(g_chart_spo2, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(g_chart_spo2, CHART_POINTS);
  lv_chart_set_range(g_chart_spo2, LV_CHART_AXIS_PRIMARY_Y, 80, 100);

  g_s_spo2 = lv_chart_add_series(g_chart_spo2, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

  for (int i = 0; i < CHART_POINTS; i++) lv_chart_set_next_value(g_chart_spo2, g_s_spo2, 0);

  lv_obj_t* hint = lv_label_create(lv_scr_act());
  lv_label_set_text(hint, "Tap to go to Status");
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);

  install_tap_handler();
}

static void build_status_screen()
{
  lv_obj_clean(lv_scr_act());

  lv_obj_t* title = lv_label_create(lv_scr_act());
  lv_label_set_text(title, "Status");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

  g_label_status = lv_label_create(lv_scr_act());
  lv_label_set_text(g_label_status, "...");
  lv_obj_align(g_label_status, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t* hint = lv_label_create(lv_scr_act());
  lv_label_set_text(hint, "Tap to return Home");
  lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -12);

  install_tap_handler();
}

static void ui_set_screen(ScreenMode s)
{
  g_screen = s;

  // Reset pointers so updates don't hit freed objects
  g_label_title = g_label_time = g_label_status = nullptr;
  g_chart_hr = g_chart_spo2 = nullptr;
  g_s_hr = g_s_spo2 = nullptr;

  if (g_screen == SCREEN_HOME) build_home_screen();
  else if (g_screen == SCREEN_HR) build_hr_screen();
  else if (g_screen == SCREEN_SPO2) build_spo2_screen();
  else build_status_screen();
}

static void ui_update_home()
{
  if (!g_label_time) return;
  char tbuf[16];
  get_time_string(tbuf, sizeof(tbuf));
  lv_label_set_text(g_label_time, tbuf);
}

static void ui_update_status()
{
  if (!g_label_status) return;
  char buf[128];
  snprintf(buf, sizeof(buf), "%s\n%s\nHR=%d  SpO2=%d",
           wifi_state_str(), ble_state_str(), (int)g_hr, (int)g_spo2);
  lv_label_set_text(g_label_status, buf);
}

static void ui_push_hr()
{
  if (!g_chart_hr || !g_s_hr) return;
  int v = (g_hr > 0) ? g_hr : 0;
  lv_chart_set_next_value(g_chart_hr, g_s_hr, (lv_coord_t)v);
  lv_chart_refresh(g_chart_hr);
}

static void ui_push_spo2()
{
  if (!g_chart_spo2 || !g_s_spo2) return;
  int v = (g_spo2 > 0) ? g_spo2 : 0;
  lv_chart_set_next_value(g_chart_spo2, g_s_spo2, (lv_coord_t)v);
  lv_chart_refresh(g_chart_spo2);
}

// ------------------- Touch handling -------------------
static void on_screen_tap(lv_event_t* e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  if (g_screen == SCREEN_HOME)      ui_set_screen(SCREEN_HR);
  else if (g_screen == SCREEN_HR)   ui_set_screen(SCREEN_SPO2);
  else if (g_screen == SCREEN_SPO2) ui_set_screen(SCREEN_STATUS);
  else                              ui_set_screen(SCREEN_HOME);
}

static void install_tap_handler()
{
  lv_obj_t* btn = lv_btn_create(lv_scr_act());
  lv_obj_remove_style_all(btn);
  lv_obj_set_size(btn, LV_HOR_RES, LV_VER_RES);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(btn, on_screen_tap, LV_EVENT_CLICKED, nullptr);
  lv_obj_move_foreground(btn);
}

// ------------------- Arduino -------------------
void setup()
{
  Serial.begin(115200);

  // LVGL init
  lv_init();

  // XIAO round screen init (driver provides display + touch)
  screen_rotation = 0;       // 0..3
  lv_xiao_disp_init();
  lv_xiao_touch_init();

  // Networking
  wifi_start();

  // BLE: start scanning for the Pi peripheral
  ble_start_scan();

  // Start UI
  ui_set_screen(SCREEN_HOME);

  g_last_lv_tick = millis();
}

void loop()
{
  // LVGL tick + handler
  uint32_t now = millis();
  uint32_t dt = now - g_last_lv_tick;
  if (dt) {
    lv_tick_inc(dt);
    g_last_lv_tick = now;
  }
  lv_timer_handler();
  delay(5);

  // Periodic BLE scan retry if not connected
  static uint32_t lastBleRetry = 0;
  if (!g_ble_connected && (millis() - lastBleRetry) > 5000) {
    lastBleRetry = millis();
    if (g_scan && !g_ble_scanning) {
      ble_start_scan();
    }
  }

  // UI updates
  static uint32_t lastHome = 0;
  static uint32_t lastGraph = 0;
  static uint32_t lastStatus = 0;

  if (g_screen == SCREEN_HOME && millis() - lastHome > 250) {
    lastHome = millis();
    ui_update_home();
    install_tap_handler();
  }

  if (g_screen == SCREEN_HR && millis() - lastGraph > 120) {
    lastGraph = millis();
    ui_push_hr();
    install_tap_handler();
  }

  if (g_screen == SCREEN_SPO2 && millis() - lastGraph > 120) {
    lastGraph = millis();
    ui_push_spo2();
    install_tap_handler();
  }

  if (g_screen == SCREEN_STATUS && millis() - lastStatus > 500) {
    lastStatus = millis();
    ui_update_status();
    install_tap_handler();
  }
}
