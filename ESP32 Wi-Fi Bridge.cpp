#include <WiFi.h>
#include <WebServer.h>

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
float heartRate = 0.0;
float spo2 = 0.0;
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
  json += "\"heartRate\":" + String(heartRate, 1) + ",";
  json += "\"spo2\":" + String(spo2, 1) + ",";
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

// =========================
// Update values from Pi/BLE
// Call this when BLE data arrives
// =========================
void updateSensorData(float hr, float ox, float temp) {
  heartRate = hr;
  spo2 = ox;
  temperature = temp;
  piStatus = "Connected";
  lastDataMillis = millis();

  // TODO:
  // also update your LCD here if needed
}

// =========================
// Demo data generator
// Remove after BLE is working
// =========================
void updateDemoData() {
  static unsigned long lastDemo = 0;
  if (millis() - lastDemo >= 2000) {
    lastDemo = millis();

    static float hr = 78.0;
    static float ox = 98.0;
    static float temp = 98.6;

    hr += random(-2, 3);
    ox += random(-1, 2) * 0.1;
    temp += random(-1, 2) * 0.1;

    if (hr < 60) hr = 60;
    if (hr > 110) hr = 110;
    if (ox < 94) ox = 94;
    if (ox > 100) ox = 100;
    if (temp < 97.5) temp = 97.5;
    if (temp > 100.0) temp = 100.0;

    updateSensorData(hr, ox, temp);
  }
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
}