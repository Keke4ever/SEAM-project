#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

//Libraries for the screen:
#define USE_ARDUINO_GFX_LIBRARY

#ifndef BLACK
#define BLACK 0x0000
#endif

#include <Arduino.h>
#include <lvgl.h>
#include "lv_xiao_round_screen.h"
 
BLEClient* pClient = nullptr;

// Variables to store parsed values
volatile int heartRate = 0;
volatile int SpO2 = 0;
boolean connected_status = false;
static uint32_t last_notify=0;
static uint32_t Timeout=10000;
static lv_obj_t* status_label = nullptr;
volatile bool update_UI =false;



enum ScreenState {
  SCREEN_HELLO,
  SCREEN_STATUS
};

ScreenState current_screen = SCREEN_HELLO;


// ---------------- HELLO SCREEN ----------------
void build_hello_screen() {

  lv_obj_clean(lv_scr_act());

  // Create ONE label that we will update
  status_label = lv_label_create(lv_scr_act());
  lv_label_set_text(status_label, "HR: --\nSpO2: --");
  lv_obj_center(status_label);
}
 
// Target UUIDs
static BLEUUID serviceUUID("6d4f9c2a-1c2b-4b3d-9b9c-3e3a7f7b2b10");
static BLEUUID charUUID("c3a9f0a1-7f22-4c1a-9a8e-67e3c3c38a11");
 
static boolean doConnect = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice = nullptr;
 
// -------------------- Notify Callback --------------------
static void notifyCallback(
  BLERemoteCharacteristic* chr,
  uint8_t* data,
  size_t len,
  bool isNotify)
{
  (void)chr;
  (void)isNotify;
  
  std::string s((const char*)data, (const char*)data + len);
 
  Serial.print("Raw received: ");
  Serial.println(s.c_str());
  last_notify=millis();
  int commaIndex = s.find(',');
 
  if (commaIndex != std::string::npos) {
 
    int hr = atoi(s.substr(0, commaIndex).c_str());
    int spo2 = atoi(s.substr(commaIndex + 1).c_str());
 
    if (hr >= 0 && hr <= 250)
      heartRate = hr;
 
    if (spo2 >= 0 && spo2 <= 100)
      SpO2 = spo2;

    update_UI= true;
 
    Serial.print("Parsed -> HR: ");
    Serial.print(heartRate);
    Serial.print("  SpO2: ");
    Serial.println(SpO2);
    
  }
  else {
    Serial.println("Invalid format (expected int,int)");
  }
  
}
 
// -------------------- Client Callback --------------------
class MyClientCallback : public BLEClientCallbacks {
  void onDisconnect(BLEClient* pclient) {
    
    Serial.println("Disconnected from server (callback)");
    connected_status = false;
  }
};
 
// -------------------- Connect Function --------------------
bool connectToServer() {
 
  Serial.println("Connecting...");
 
  if (pClient != nullptr) {
    
    if (pClient->isConnected()) {
      pClient->disconnect();
      delay(300);
    }
    delete pClient;
    pClient = nullptr;
  }
 
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
 
  if (!pClient->connect(myDevice)) {
    Serial.println("Connection failed");
    return false;
  }
 
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
 
  if (pRemoteService == nullptr) {
    Serial.println("Service not found");
    pClient->disconnect();
    return false;
  }
 
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
 
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Characteristic not found");
    pClient->disconnect();
    return false;
  }
 
  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);
 
  Serial.println("Connected and subscribed!");
  connected_status = true;
 
  return true;
}
 
// -------------------- Scan Callback --------------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
 
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(serviceUUID)) {
 
      Serial.println("Found target device!");
 
      BLEDevice::getScan()->stop();
 
      
      if (myDevice != nullptr) {
        delete myDevice;
        myDevice = nullptr;
      }
 
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      
    }
  }
};
 
// -------------------- BLE Scan --------------------
static bool ble_initialized = false; 
static uint32_t last_scan = 0;
 
static void ble_start_scan() {
 

  if (!ble_initialized) {
    BLEDevice::init("");
    ble_initialized = true;
  }
 
  
  BLEDevice::getScan()->clearResults();
 
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, true);
 
  
  last_scan = millis();
}
 
// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  lv_init();
  lv_xiao_disp_init();
  lv_xiao_touch_init();
  build_hello_screen();

  Serial.println("Starting BLE Client");
  ble_start_scan();
}
 
// -------------------- Loop --------------------
void loop() {

  lv_timer_handler();
  delay(5);
 
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Ready to receive data...");
    }
    doConnect = false;
  }
 
  
  if (pClient != nullptr && pClient->isConnected()) {
    if(last_notify!=0 && (millis()- last_notify)> Timeout){
      last_notify=0;
      connected_status=false;
      Serial.println("10 seconds from last update disconnecting");
      pClient->disconnect();
    } else{
      connected_status=true;
    }
  }
  else if (!doConnect) {
    if (connected_status) {
      Serial.println("Detected disconnection (poll)");
      connected_status = false;
    }
 
    if (millis() - last_scan > 6000) {
      Serial.println("Scanning....");
      ble_start_scan();
    }
  }
 
  
  if(update_UI && status_label != nullptr){
    char buffer[50];
    sprintf(buffer, "HR: %d\nSpO2: %d", heartRate, SpO2);
    Serial.println(buffer);
    lv_label_set_text(status_label, buffer);
    lv_obj_invalidate(status_label);
    lv_refr_now(NULL);
    update_UI=false;
  }
  
  
}