#include "Arduino.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <LittleFS.h>
#include "esp_task_wdt.h"
#include "config.h"
#include "storage.h"
#include "portal.h"

// ===== TEST CONFIG =====
#define TEST_RECORD_COUNT   12
#define TEST_CYCLE_SECONDS  60

// ===== GLOBALS =====
DeviceConfig    deviceConfig;
WiFiCredentials wifiCreds;

WiFiClient           mqttWifiClient;
Adafruit_MQTT_Client mqtt(&mqttWifiClient, AIO_SERVER, AIO_PORT, "", "");
char TOPIC_TRUCK[80];
char TOPIC_BATT[80];
char TOPIC_DEBUG[80];
Adafruit_MQTT_Publish* feedTruckVoltage = nullptr;
Adafruit_MQTT_Publish* feedBattVoltage  = nullptr;
Adafruit_MQTT_Publish* feedDebug        = nullptr;

bool fsReady    = false;
bool mqttReady  = false;
int  cycleCount = 0;

unsigned long lastCycleMs = 0;

// ===== WATCHDOG =====
void feedWatchdog() { esp_task_wdt_reset(); }

// ===== FAKE DATA =====
float fakeTruckVolts() { return 14.100 + (random(0, 200) / 1000.0); }
float fakeBattVolts()  { return 3.300  + (random(0, 100) / 1000.0); }

String fakeTimestamp(int index) {
  int totalSeconds = index * 60;
  int hh = (totalSeconds / 3600) % 24;
  int mm = (totalSeconds / 60) % 60;
  int ss = totalSeconds % 60;
  char buf[20];
  snprintf(buf, sizeof(buf), "2026-03-18 %02d:%02d:%02d", hh, mm, ss);
  return String(buf);
}

// ===== LITTLEFS =====
bool initFS() {
  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS failed");
    return false;
  }
  File testFile = LittleFS.open("/fstest", "w");
  if (!testFile) {
    Serial.println("LittleFS not writable — reformatting...");
    LittleFS.end();
    LittleFS.format();
    if (!LittleFS.begin(true)) {
      Serial.println("ERROR: LittleFS failed after format");
      return false;
    }
  } else {
    testFile.close();
    LittleFS.remove("/fstest");
  }
  Serial.println("LittleFS OK");
  return true;
}

// ===== MQTT SETUP =====
void setupMQTT() {
  if (strlen(deviceConfig.aioUsername) == 0) {
    Serial.println("No AIO credentials — MQTT disabled");
    mqttReady = false;
    return;
  }

  mqtt.~Adafruit_MQTT_Client();
  new (&mqtt) Adafruit_MQTT_Client(&mqttWifiClient, AIO_SERVER, AIO_PORT,
                                    deviceConfig.aioUsername, deviceConfig.aioKey);

  snprintf(TOPIC_TRUCK, sizeof(TOPIC_TRUCK), "%s/feeds/truck-voltage",  deviceConfig.aioUsername);
  snprintf(TOPIC_BATT,  sizeof(TOPIC_BATT),  "%s/feeds/device-voltage", deviceConfig.aioUsername);
  snprintf(TOPIC_DEBUG, sizeof(TOPIC_DEBUG), "%s/feeds/debug-messages", deviceConfig.aioUsername);

  delete feedTruckVoltage;
  delete feedBattVoltage;
  delete feedDebug;

  feedTruckVoltage = new Adafruit_MQTT_Publish(&mqtt, TOPIC_TRUCK);
  feedBattVoltage  = new Adafruit_MQTT_Publish(&mqtt, TOPIC_BATT);
  feedDebug        = new Adafruit_MQTT_Publish(&mqtt, TOPIC_DEBUG);

  mqttReady = true;
  Serial.println("MQTT configured for: " + String(deviceConfig.aioUsername));
}

// ===== WIFI CONNECT =====
bool connectWiFi() {
  if (wifiCreds.count == 0) {
    Serial.println("No WiFi credentials");
    return false;
  }

  Serial.println("Connecting to WiFi...");
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(portalActive ? WIFI_AP_STA : WIFI_STA);
  delay(500);

  // Scan
  Serial.println("Scanning...");
  int found = WiFi.scanNetworks();
  Serial.println("Networks found: " + String(found));

  int bestIndex = -1;
  int bestRSSI  = -999;

  for (int i = 0; i < found; i++) {
    String scannedSSID = WiFi.SSID(i);
    int    scannedRSSI = WiFi.RSSI(i);
    Serial.println("  " + scannedSSID + " (" + String(scannedRSSI) + " dBm)");
    for (int n = 0; n < wifiCreds.count; n++) {
      if (scannedSSID == wifiCreds.networks[n].ssid && scannedRSSI > bestRSSI) {
        bestRSSI  = scannedRSSI;
        bestIndex = n;
      }
    }
  }
  WiFi.scanDelete();

  if (bestIndex < 0) {
    Serial.println("No known networks found");
    return false;
  }

  Serial.println("Best: " + String(wifiCreds.networks[bestIndex].ssid) +
                 " (" + String(bestRSSI) + " dBm)");

  // Reset radio after scan — only if portal not running
  if (!portalActive) {
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);
    delay(1000);
  }

  for (int attempt = 1; attempt <= WIFI_ATTEMPTS_PER_NETWORK; attempt++) {
    Serial.println("Attempt " + String(attempt) + "/" + String(WIFI_ATTEMPTS_PER_NETWORK));
    WiFi.begin(wifiCreds.networks[bestIndex].ssid.c_str(),
               wifiCreds.networks[bestIndex].pass.c_str());

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED && dots < 20) {
      delay(500);
      feedWatchdog();
      dots++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected: " + WiFi.localIP().toString() +
                     " RSSI:" + String(WiFi.RSSI()) + " dBm");
      return true;
    }

    Serial.println("Failed — status: " + String(WiFi.status()));
    WiFi.disconnect(false);
    if (attempt < WIFI_ATTEMPTS_PER_NETWORK) delay(1000);
  }

  return false;
}

void disconnectWiFi() {
  if (mqttReady) mqtt.disconnect();
  WiFi.disconnect(true);
  if (!portalActive) {
    WiFi.mode(WIFI_OFF);
  }
  delay(500);
}

// ===== MQTT CONNECT =====
bool connectMQTT() {
  if (!mqttReady) return false;
  Serial.println("Connecting to Adafruit IO...");
  int attempts = 0;
  while (mqtt.connect() != 0 && attempts < 3) {
    mqtt.disconnect();
    feedWatchdog();
    delay(2000);
    attempts++;
  }
  if (mqtt.connected()) { Serial.println("Adafruit IO connected"); return true; }
  Serial.println("Adafruit IO FAILED");
  return false;
}

// ===== GOOGLE SHEETS =====
bool sendToGoogleSheets(String compactData) {
  Serial.println("Sending to Google Sheets...");
  Serial.println("Device: [" + String(deviceConfig.deviceName) + "]");

  String payload = "device=" + String(deviceConfig.deviceName) + "&d=" + compactData;
  payload.replace(" ", "%20");

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  String url = String(GS_SCRIPT_URL) + "?" + payload;
  Serial.println("URL length: " + String(url.length()));

  HTTPClient http;
  http.begin(secureClient, url);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  int httpCode = http.GET();
  feedWatchdog();

  String response = http.getString();
  http.end();

  Serial.println("HTTP " + String(httpCode) + " — " + response);
  return response.indexOf("\"ok\"") >= 0;
}

// ===== HANDLE PORTAL UPDATES =====
bool handlePortalUpdates() {
  if (!credsUpdated && !configUpdated) return false;

  credsUpdated  = false;
  configUpdated = false;
  loadConfig(deviceConfig);
  loadWiFiCredentials(wifiCreds);
  setupMQTT();
  Serial.println("Credentials updated via portal — retrying WiFi");

  if (connectWiFi()) {
    stopPortal();
    Serial.println("WiFi connected after portal update");

    // Run an immediate upload cycle
    float truckV = fakeTruckVolts();
    float battV  = fakeBattVolts();
    String compact = fakeTimestamp(0) + "," + String(truckV, 3) + "," +
                     String(battV, 3) + ",1";
    sendToGoogleSheets(compact);

    if (connectMQTT()) {
      feedTruckVoltage->publish(truckV);
      feedBattVoltage->publish(battV);
      String dbg = "PORTAL_CONNECT RSSI:" + String(WiFi.RSSI()) +
                   " Device:" + String(deviceConfig.deviceName);
      feedDebug->publish(dbg.c_str());
      mqtt.disconnect();
    }
    disconnectWiFi();
    return true;
  }

  Serial.println("New credentials failed — portal remains open");
  return false;
}

// ===== RUN TEST CYCLE =====
void runTestCycle() {
  cycleCount++;
  Serial.println("\n=============================");
  Serial.println("TEST CYCLE #" + String(cycleCount));
  Serial.println("Device: [" + String(deviceConfig.deviceName) + "]");
  Serial.println("WiFi networks: " + String(wifiCreds.count));
  for (int i = 0; i < wifiCreds.count; i++) {
    Serial.println("  [" + String(i) + "] " + wifiCreds.networks[i].ssid);
  }
  Serial.println("Portal active: " + String(portalActive ? "YES" : "NO"));
  Serial.println("=============================");

  // Generate fake records
  String compact = "";
  float lastTruck = 0, lastBatt = 0;
  for (int i = 0; i < TEST_RECORD_COUNT; i++) {
    lastTruck = fakeTruckVolts();
    lastBatt  = fakeBattVolts();
    if (i > 0) compact += "|";
    compact += fakeTimestamp(i) + "," + String(lastTruck, 3) + "," +
               String(lastBatt, 3) + ",1";
  }

  if (!connectWiFi()) {
    Serial.println("WiFi failed — " + String(portalActive ? "portal already running" : "starting portal"));
    if (!portalActive) {
      WiFi.mode(WIFI_AP_STA);
      delay(200);
      startPortal(deviceConfig, wifiCreds);
    }
    return;
  }

  // If portal was running and we connected, stop it
  if (portalActive) stopPortal();

  bool sheetsOk = sendToGoogleSheets(compact);
  Serial.println("Sheets: " + String(sheetsOk ? "OK" : "FAILED"));

  if (connectMQTT()) {
    bool ok1 = feedTruckVoltage->publish(lastTruck);
    bool ok2 = feedBattVoltage->publish(lastBatt);
    String dbg = "TEST#" + String(cycleCount) +
                 " RSSI:" + String(WiFi.RSSI()) +
                 " Truck:" + String(lastTruck, 3) +
                 " Batt:" + String(lastBatt, 3) +
                 " Sheets:" + String(sheetsOk ? "OK" : "FAIL") +
                 " Device:" + String(deviceConfig.deviceName);
    bool ok3 = feedDebug->publish(dbg.c_str());
    Serial.println("MQTT truck:" + String(ok1 ? "OK" : "FAIL") +
                   " batt:" + String(ok2 ? "OK" : "FAIL") +
                   " debug:" + String(ok3 ? "OK" : "FAIL"));
    mqtt.disconnect();
  }

  disconnectWiFi();
  Serial.println("Cycle complete — next in " + String(TEST_CYCLE_SECONDS) + "s");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(120, false);
  delay(3000);

  Serial.println("\n=== VoltMon Portal Test ===");

  fsReady = initFS();
  if (!fsReady) {
    Serial.println("FATAL: LittleFS failed");
    while (1) delay(1000);
  }

  loadConfig(deviceConfig);
  loadWiFiCredentials(wifiCreds);

  // Seed from secrets on first flash if configured
#ifdef SEED_FROM_SECRETS
  if (!deviceConfig.configured) {
    Serial.println("Seeding from secrets.h...");
    seedFromSecrets(deviceConfig, wifiCreds);
    strlcpy(deviceConfig.deviceName, DEVICE_NAME, sizeof(deviceConfig.deviceName));
    saveConfig(deviceConfig);
  }
#endif

  Serial.println("Device: [" + String(deviceConfig.deviceName) + "]");
  Serial.println("Configured: " + String(deviceConfig.configured ? "YES" : "NO"));
  Serial.println("WiFi networks: " + String(wifiCreds.count));
  for (int i = 0; i < wifiCreds.count; i++) {
    Serial.println("  [" + String(i) + "] " + wifiCreds.networks[i].ssid);
  }

  setupMQTT();

  // If not configured, start portal immediately and wait
  if (!deviceConfig.configured) {
    Serial.println("Not configured — starting VoltMon-Initialize portal");
    WiFi.mode(WIFI_AP_STA);
    delay(200);
    startPortal(deviceConfig, wifiCreds);

    while (!deviceConfig.configured) {
      if (credsUpdated || configUpdated) {
        loadConfig(deviceConfig);
        loadWiFiCredentials(wifiCreds);
        setupMQTT();
        credsUpdated  = false;
        configUpdated = false;
        if (deviceConfig.configured) {
          Serial.println("Device configured via portal — continuing");
          stopPortal();
          break;
        }
      }
      delay(500);
      feedWatchdog();
    }
  }

  Serial.println("Starting test loop...");
  lastCycleMs = millis();
}

// ===== LOOP =====
void loop() {
  // Process DNS for captive portal
  portalLoop();

  // Check for portal credential updates
  if (portalActive) {
    handlePortalUpdates();
  }

  // Don't run test cycle until configured
  if (!deviceConfig.configured) {
    delay(100);
    feedWatchdog();
    return;
  }

  // Don't interrupt portal if someone is connected
  if (portalActive && portalHasClients()) {
    delay(100);
    feedWatchdog();
    return;
  }

  if (millis() - lastCycleMs >= (unsigned long)TEST_CYCLE_SECONDS * 1000) {
    runTestCycle();
    lastCycleMs = millis();
  }

  delay(100);
  feedWatchdog();
}