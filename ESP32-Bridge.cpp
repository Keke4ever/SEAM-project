/*
  SEAM ESP32 Bridge
  -----------------
  Purpose:
    - Receive sensor data from Raspberry Pi over UART
    - Send that data to mobile app over BLE notifications
    - Receive commands from mobile app over BLE writes
    - Forward commands back to Raspberry Pi over UART

  Expected UART line format from Raspberry Pi:
    {"hr":78,"spo2":98,"motion":"walking"}

  Expected BLE write format from mobile app:
    Any plain-text command, for example:
    START
    STOP
    ALERT_TEST
    {"cmd":"set_mode","value":"monitor"}

  Hardware:
    - ESP32 / ESP32-S3
    - Raspberry Pi TX -> ESP32 RX pin
    - Raspberry Pi RX -> ESP32 TX pin
    - Common GND required

  Notes:
    - Change UART pins below to match your wiring
    - Change BLE device name and UUIDs if needed
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =========================
// User Config
// =========================
#define UART_BAUD_RATE 115200

// Adjust these pins for your board
// Example for many ESP32 boards:
// RX = pin receiving from Pi TX
// TX = pin sending to Pi RX
static const int UART_RX_PIN = 18;
static const int UART_TX_PIN = 17;

// BLE settings
static const char* DEVICE_NAME = "SEAM-Bridge";

// Custom BLE UUIDs
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define TX_CHAR_UUID        "12345678-1234-1234-1234-1234567890ac" // ESP32 -> phone notify
#define RX_CHAR_UUID        "12345678-1234-1234-1234-1234567890ad" // phone -> ESP32 write

// =========================
// Globals
// =========================
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;

bool deviceConnected = false;
bool oldDeviceConnected = false;

String uartBuffer = "";

// Use UART1 for Pi bridge
HardwareSerial PiSerial(1);

// =========================
// BLE Server Callbacks
// =========================
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[BLE] Mobile app connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[BLE] Mobile app disconnected");
  }
};

// =========================
// BLE RX Callbacks
// =========================
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = pCharacteristic->getValue().c_str();

    if (rxValue.length() > 0) {
      Serial.print("[BLE <- APP] ");
      Serial.println(rxValue);

      // Forward app command to Raspberry Pi over UART
      PiSerial.println(rxValue);

      Serial.print("[UART -> PI] ");
      Serial.println(rxValue);
    }
  }
};

// =========================
// BLE Setup
// =========================
void setupBLE() {
  BLEDevice::init(DEVICE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // TX characteristic: notify sensor data to app
  pTxCharacteristic = pService->createCharacteristic(
    TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX characteristic: receive commands from app
  pRxCharacteristic = pService->createCharacteristic(
    RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising started");
}

// =========================
// UART Line Handling
// =========================
void handlePiUART() {
  while (PiSerial.available()) {
    char c = (char)PiSerial.read();

    if (c == '\n') {
      uartBuffer.trim();

      if (uartBuffer.length() > 0) {
        Serial.print("[UART <- PI] ");
        Serial.println(uartBuffer);

        // Notify connected mobile app
        if (deviceConnected && pTxCharacteristic != nullptr) {
          pTxCharacteristic->setValue(uartBuffer.c_str());
          pTxCharacteristic->notify();

          Serial.print("[BLE -> APP] ");
          Serial.println(uartBuffer);
        }
      }

      uartBuffer = "";
    } else {
      uartBuffer += c;

      // Safety limit in case malformed stream arrives
      if (uartBuffer.length() > 512) {
        Serial.println("[WARN] UART buffer overflow, clearing");
        uartBuffer = "";
      }
    }
  }
}

// =========================
// Connection Maintenance
// =========================
void handleBLEReconnect() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[BLE] Restarted advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==================================");
  Serial.println("SEAM ESP32 Bridge Starting...");
  Serial.println("==================================");

  // UART bridge to Raspberry Pi
  PiSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.print("[UART] Started on RX=");
  Serial.print(UART_RX_PIN);
  Serial.print(" TX=");
  Serial.print(UART_TX_PIN);
  Serial.print(" @ ");
  Serial.println(UART_BAUD_RATE);

  setupBLE();
}

// =========================
// Main Loop
// =========================
void loop() {
  handlePiUART();
  handleBLEReconnect();
}