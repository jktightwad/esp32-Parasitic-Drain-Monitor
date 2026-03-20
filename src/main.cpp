#include "Arduino.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include "esp_adc_cal.h"
#include <Wire.h>
#include <RTClib.h>
#include <LittleFS.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <time.h>
#include "esp_task_wdt.h"
#include "config.h"
#include "storage.h"
#include "portal.h"
#include "ble_server.h"

// ===== GLOBAL STATE =====
DeviceConfig    deviceConfig;
WiFiCredentials wifiCreds;

int  recordCount     = 0;
bool ntpSynced       = false;
int  wakeCount       = 0;
int  bleFailCount    = 0;
long lastWiFiAttempt = 0;

esp_adc_cal_characteristics_t adc_chars;
RTC_DS3231 rtc;

// MQTT — used only during WiFi fallback
WiFiClient           mqttWifiClient;
Adafruit_MQTT_Client mqtt(&mqttWifiClient, AIO_SERVER, AIO_PORT, "", "");
char TOPIC_TRUCK[80];
char TOPIC_BATT[80];
char TOPIC_DEBUG[80];
Adafruit_MQTT_Publish* feedTruckVoltage = nullptr;
Adafruit_MQTT_Publish* feedBattVoltage  = nullptr;
Adafruit_MQTT_Publish* feedDebug        = nullptr;

bool fsReady   = false;
bool mqttReady = false;

// ===== WATCHDOG =====
void feedWatchdog() {
  esp_task_wdt_reset();
}

// ===== VERSION PARSING =====
int parseVersion(String v) {
  v.trim();
  int major = 0, minor = 0, patch = 0;
  int first  = v.indexOf('.');
  int second = v.indexOf('.', first + 1);
  if (first < 0) {
    major = v.toInt();
  } else if (second < 0) {
    major = v.substring(0, first).toInt();
    minor = v.substring(first + 1).toInt();
  } else {
    major = v.substring(0, first).toInt();
    minor = v.substring(first + 1, second).toInt();
    patch = v.substring(second + 1).toInt();
  }
  return major * 10000 + minor * 100 + patch;
}

// ===== DEBUG LOGGING =====
void debugLog(String msg) {
  Serial.println(msg);
  if (!fsReady) return;

  if (LittleFS.exists(DEBUG_FILE)) {
    File f = LittleFS.open(DEBUG_FILE, "r");
    if (f) {
      if (f.size() > MAX_DEBUG_SIZE) {
        f.close();
        LittleFS.remove(DEBUG_FILE);
      } else {
        f.close();
      }
    }
  }

  File f = LittleFS.open(DEBUG_FILE, "a");
  if (f) {
    DateTime now = rtc.now();
    char buf[20];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second());
    f.print(buf);
    f.print(" | ");
    f.println(msg);
    f.close();
  }
}

void dumpDebugLog() {
  if (!fsReady || !LittleFS.exists(DEBUG_FILE)) {
    Serial.println("No debug log found");
    return;
  }
  File f = LittleFS.open(DEBUG_FILE, "r");
  if (!f) return;
  Serial.println("=== DEBUG LOG ===");
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.println("\n=== END DEBUG LOG ===");
}

void dumpPendingFile() {
  if (!fsReady || !LittleFS.exists(PENDING_FILE)) {
    Serial.println("No pending file found");
    return;
  }
  File f = LittleFS.open(PENDING_FILE, "r");
  if (!f) return;
  Serial.println("=== PENDING FILE ===");
  while (f.available()) Serial.write(f.read());
  f.close();
  Serial.println("\n=== END PENDING FILE ===");
}

// ===== MQTT SETUP =====
void setupMQTT() {
  if (strlen(deviceConfig.aioUsername) == 0) {
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
  debugLog("MQTT configured for: " + String(deviceConfig.aioUsername));
}

// ===== STATE PERSISTENCE =====
void loadState() {
  if (!fsReady || !LittleFS.exists(STATE_FILE)) {
    debugLog("No state file — starting fresh");
    return;
  }

  File f = LittleFS.open(STATE_FILE, "r");
  if (!f) return;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("recordCount="))
      recordCount = line.substring(12).toInt();
    else if (line.startsWith("wakeCount="))
      wakeCount = line.substring(10).toInt();
    else if (line.startsWith("ntpSynced="))
      ntpSynced = line.substring(10).toInt() == 1;
    else if (line.startsWith("bleFailCount="))
      bleFailCount = line.substring(13).toInt();
    else if (line.startsWith("lastWiFiAttempt="))
      lastWiFiAttempt = line.substring(16).toInt();
  }
  f.close();

  debugLog("State loaded — wakes=" + String(wakeCount) +
           " records=" + String(recordCount) +
           " bleFailCount=" + String(bleFailCount));
}

void saveState() {
  if (!fsReady) return;

  File f = LittleFS.open(STATE_FILE, "w");
  if (!f) {
    debugLog("ERROR: Could not save state");
    return;
  }
  f.println("recordCount="     + String(recordCount));
  f.println("wakeCount="       + String(wakeCount));
  f.println("ntpSynced="       + String(ntpSynced ? 1 : 0));
  f.println("bleFailCount="    + String(bleFailCount));
  f.println("lastWiFiAttempt=" + String(lastWiFiAttempt));
  f.close();
}

// ===== TRUCK VOLTAGE CALIBRATION =====
float applyTruckCalibration(float rawVolts) {
  if (rawVolts <= TRUCK_CAL_TABLE[0][0]) {
    float slope = (TRUCK_CAL_TABLE[1][1] - TRUCK_CAL_TABLE[0][1]) /
                  (TRUCK_CAL_TABLE[1][0] - TRUCK_CAL_TABLE[0][0]);
    return TRUCK_CAL_TABLE[0][1] + slope * (rawVolts - TRUCK_CAL_TABLE[0][0]);
  }

  if (rawVolts >= TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE - 1][0]) {
    float slope = (TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-1][1] - TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-2][1]) /
                  (TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-1][0] - TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-2][0]);
    return TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-1][1] +
           slope * (rawVolts - TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-1][0]);
  }

  for (int i = 0; i < TRUCK_CAL_TABLE_SIZE - 1; i++) {
    if (rawVolts >= TRUCK_CAL_TABLE[i][0] && rawVolts <= TRUCK_CAL_TABLE[i+1][0]) {
      float slope = (TRUCK_CAL_TABLE[i+1][1] - TRUCK_CAL_TABLE[i][1]) /
                    (TRUCK_CAL_TABLE[i+1][0] - TRUCK_CAL_TABLE[i][0]);
      return TRUCK_CAL_TABLE[i][1] + slope * (rawVolts - TRUCK_CAL_TABLE[i][0]);
    }
  }
  return rawVolts;
}

// ===== BATTERY VOLTAGE CALIBRATION =====
float applyBattCalibration(float rawVolts) {
  float slope = (BATT_CAL_TABLE[1][1] - BATT_CAL_TABLE[0][1]) /
                (BATT_CAL_TABLE[1][0] - BATT_CAL_TABLE[0][0]);

  if (rawVolts <= BATT_CAL_TABLE[0][0])
    return BATT_CAL_TABLE[0][1] + slope * (rawVolts - BATT_CAL_TABLE[0][0]);

  if (rawVolts >= BATT_CAL_TABLE[BATT_CAL_TABLE_SIZE-1][0])
    return BATT_CAL_TABLE[BATT_CAL_TABLE_SIZE-1][1] +
           slope * (rawVolts - BATT_CAL_TABLE[BATT_CAL_TABLE_SIZE-1][0]);

  return BATT_CAL_TABLE[0][1] + slope * (rawVolts - BATT_CAL_TABLE[0][0]);
}

// ===== ADC =====
float readVoltage(int sourcePin, int adcPin, float dividerRatio) {
  digitalWrite(sourcePin, HIGH);
  delayMicroseconds(100);

  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(adcPin);
    delayMicroseconds(50);
  }
  uint32_t raw        = sum / ADC_SAMPLES;
  uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw, &adc_chars);

  digitalWrite(sourcePin, LOW);
  return (millivolts / 1000.0) * dividerRatio;
}

float readBattVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(PIN_BATT_ADC);
    delayMicroseconds(50);
  }
  uint32_t raw        = sum / ADC_SAMPLES;
  uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  float rawVolts      = (millivolts / 1000.0) * BATT_DIVIDER_RATIO;
  return applyBattCalibration(rawVolts);
}

// ===== TIMESTAMP =====
String getTimestamp(DateTime now) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
    now.year(), now.month(), now.day(),
    now.hour(), now.minute(), now.second());
  return String(buf);
}

// ===== FILE =====
void writePending(DateTime now, float truckVolts, float battVolts, bool charging) {
  if (!fsReady) return;

  if (LittleFS.exists(PENDING_FILE)) {
    File f = LittleFS.open(PENDING_FILE, "r");
    if (f) {
      int count = 0;
      while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) count++;
      }
      f.close();

      if (count >= MAX_PENDING_RECORDS) {
        debugLog("Pending full — archiving oldest records");
        f = LittleFS.open(PENDING_FILE, "r");
        String lines[MAX_PENDING_RECORDS];
        int idx = 0;
        while (f.available() && idx < MAX_PENDING_RECORDS) {
          lines[idx] = f.readStringUntil('\n');
          lines[idx].trim();
          if (lines[idx].length() > 0) idx++;
        }
        f.close();

        int keepFrom = MAX_PENDING_RECORDS / 2;
        File archive = LittleFS.open(ARCHIVE_FILE, "a");
        if (archive) {
          for (int i = 0; i < keepFrom; i++) {
            if (lines[i].length() > 0) archive.println(lines[i]);
          }
          archive.close();
        }

        LittleFS.remove(PENDING_FILE);
        File newf = LittleFS.open(PENDING_FILE, "w");
        if (newf) {
          for (int i = keepFrom; i < idx; i++) {
            if (lines[i].length() > 0) newf.println(lines[i]);
          }
          newf.close();
        }
        debugLog("Trimmed pending to " + String(idx - keepFrom) + " records");
      }
    }
  }

  File f;
  int retries = 0;
  while (retries < 3) {
    f = LittleFS.open(PENDING_FILE, "a");
    if (f) break;
    debugLog("Pending open failed — retry " + String(retries + 1));
    delay(1000);
    retries++;
  }
  if (!f) {
    debugLog("ERROR: Could not open pending file");
    return;
  }

  f.print(getTimestamp(now));
  f.print(",");
  f.print(truckVolts, 3);
  f.print(",");
  f.print(battVolts, 3);
  f.print(",");
  f.println(charging ? "true" : "false");
  f.close();
}

void archiveRecord(String line) {
  if (!fsReady) return;
  if (LittleFS.exists(ARCHIVE_FILE)) {
    File f = LittleFS.open(ARCHIVE_FILE, "r");
    if (f) {
      if (f.size() > MAX_ARCHIVE_SIZE) {
        f.close();
        LittleFS.remove(ARCHIVE_FILE);
        debugLog("Archive rotated");
      } else {
        f.close();
      }
    }
  }
  File f = LittleFS.open(ARCHIVE_FILE, "a");
  if (f) {
    f.println(line);
    f.close();
  }
}

bool hasPendingRecords() {
  if (!fsReady || !LittleFS.exists(PENDING_FILE)) return false;
  File f = LittleFS.open(PENDING_FILE, "r");
  if (!f) return false;
  bool hasContent = f.size() > 0;
  f.close();
  return hasContent;
}

// ===== WIFI CONNECT (fallback only) =====
bool connectWiFi() {
  if (wifiCreds.count == 0) {
    debugLog("No WiFi credentials");
    return false;
  }

  debugLog("WiFi fallback attempt...");
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(portalActive ? WIFI_AP_STA : WIFI_STA);
  delay(500);

  int found = WiFi.scanNetworks();
  debugLog("WiFi scan: " + String(found) + " networks");

  int     bestRSSI[MAX_WIFI_NETWORKS];
  bool    seen[MAX_WIFI_NETWORKS];
  uint8_t bestBSSID[MAX_WIFI_NETWORKS][6];
  int     bestChannel[MAX_WIFI_NETWORKS];
  int     visibleIndices[MAX_WIFI_NETWORKS];
  int     visibleRSSI[MAX_WIFI_NETWORKS];
  uint8_t visibleBSSID[MAX_WIFI_NETWORKS][6];
  int     visibleChannels[MAX_WIFI_NETWORKS];
  int     visibleCount = 0;

  for (int i = 0; i < wifiCreds.count; i++) {
    bestRSSI[i] = -999;
    seen[i]     = false;
  }

  for (int i = 0; i < found; i++) {
    String scannedSSID = WiFi.SSID(i);
    int    scannedRSSI = WiFi.RSSI(i);
    int    scannedCh   = WiFi.channel(i);
    for (int n = 0; n < wifiCreds.count; n++) {
      if (scannedSSID == wifiCreds.networks[n].ssid && scannedRSSI > bestRSSI[n]) {
        bestRSSI[n]    = scannedRSSI;
        bestChannel[n] = scannedCh;
        memcpy(bestBSSID[n], WiFi.BSSID(i), 6);
        seen[n] = true;
      }
    }
  }
  WiFi.scanDelete();

  for (int n = 0; n < wifiCreds.count; n++) {
    if (!seen[n]) continue;
    int pos = visibleCount;
    while (pos > 0 && visibleRSSI[pos-1] < bestRSSI[n]) pos--;
    for (int j = visibleCount; j > pos; j--) {
      visibleIndices[j]  = visibleIndices[j-1];
      visibleRSSI[j]     = visibleRSSI[j-1];
      visibleChannels[j] = visibleChannels[j-1];
      memcpy(visibleBSSID[j], visibleBSSID[j-1], 6);
    }
    visibleIndices[pos]  = n;
    visibleRSSI[pos]     = bestRSSI[n];
    visibleChannels[pos] = bestChannel[n];
    memcpy(visibleBSSID[pos], bestBSSID[n], 6);
    visibleCount++;
  }

  if (visibleCount == 0) {
    debugLog("No known WiFi in range");
    return false;
  }

  if (!portalActive) {
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.mode(WIFI_STA);
    delay(1000);
  }

  // Single attempt per network — fallback only
  for (int i = 0; i < visibleCount; i++) {
    int netIdx = visibleIndices[i];
    debugLog("WiFi trying: " + wifiCreds.networks[netIdx].ssid);

    WiFi.begin(wifiCreds.networks[netIdx].ssid.c_str(),
               wifiCreds.networks[netIdx].pass.c_str());

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED && dots < 20) {
      delay(500);
      feedWatchdog();
      dots++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      debugLog("WiFi connected: " + WiFi.localIP().toString() +
               " RSSI:" + String(WiFi.RSSI()));
      return true;
    }

    WiFi.disconnect(false);
    delay(500);
  }

  debugLog("WiFi fallback failed");
  return false;
}

void disconnectWiFi() {
  if (mqttReady) mqtt.disconnect();
  WiFi.disconnect(true);
  if (!portalActive) WiFi.mode(WIFI_OFF);
  delay(500);
}

// ===== NTP =====
bool syncNTP() {
  debugLog("Syncing NTP...");
  configTime(0, 0, NTP_SERVER);

  struct tm timeinfo;
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 10) {
    delay(500);
    feedWatchdog();
    attempts++;
  }

  if (attempts >= 10) {
    debugLog("NTP sync FAILED");
    return false;
  }

  rtc.adjust(DateTime(
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec
  ));

  debugLog("RTC synced: " + getTimestamp(rtc.now()));
  return true;
}

// ===== MQTT =====
bool connectMQTT() {
  if (!mqttReady) return false;

  debugLog("Connecting to Adafruit IO...");
  int attempts = 0;
  while (mqtt.connect() != 0 && attempts < 3) {
    mqtt.disconnect();
    feedWatchdog();
    delay(2000);
    attempts++;
  }

  if (mqtt.connected()) {
    debugLog("Adafruit IO connected");
    return true;
  }

  debugLog("Adafruit IO FAILED");
  return false;
}

void publishLatestReading(float truckVolts, float battVolts) {
  if (!connectMQTT()) return;
  bool ok1 = feedTruckVoltage->publish(truckVolts);
  bool ok2 = feedBattVoltage->publish(battVolts);
  debugLog(ok1 ? "MQTT truck: " + String(truckVolts, 3) + "V" : "MQTT truck failed");
  debugLog(ok2 ? "MQTT batt: "  + String(battVolts, 3)  + "V" : "MQTT batt failed");

  if (deviceConfig.debugMode && feedDebug) {
    String msg = "WiFi_FALLBACK RSSI:" + String(WiFi.RSSI()) +
                 " Truck:" + String(truckVolts, 3) +
                 " Batt:" + String(battVolts, 3) +
                 " BLEFails:" + String(bleFailCount);
    feedDebug->publish(msg.c_str());
  }

  mqtt.disconnect();
}

// ===== GOOGLE SHEETS =====
bool sendToGoogleSheets(String compactData) {
  debugLog("Sending to Google Sheets...");

  String payload = "device=" + String(deviceConfig.deviceName) + "&d=" + compactData;
  payload.replace(" ", "%20");

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  String url = String(GS_SCRIPT_URL) + "?" + payload;

  HTTPClient http;
  http.begin(secureClient, url);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  int httpCode = http.GET();
  feedWatchdog();

  String response = http.getString();
  http.end();

  debugLog("HTTP " + String(httpCode) + " — " + response.substring(0, 80));
  return response.indexOf("\"ok\"") >= 0;
}

// ===== UPLOAD PENDING VIA WIFI =====
void uploadPendingViaWiFi() {
  if (!fsReady || !LittleFS.exists(PENDING_FILE)) {
    debugLog("No pending records");
    return;
  }

  File f = LittleFS.open(PENDING_FILE, "r");
  if (!f) return;

  String lines[MAX_PENDING_RECORDS];
  int totalCount = 0;

  while (f.available() && totalCount < MAX_PENDING_RECORDS) {
    feedWatchdog();
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 + 1);
    int comma3 = line.lastIndexOf(',');
    if (comma1 < 0 || comma2 < 0 || comma3 < 0 || comma1 == comma3) continue;
    lines[totalCount++] = line;
  }
  f.close();

  if (totalCount == 0) return;
  debugLog("WiFi upload: " + String(totalCount) + " records");

  const int BATCH_SIZE = 40;
  int sent = 0;

  while (sent < totalCount) {
    feedWatchdog();
    int batchEnd = min(sent + BATCH_SIZE, totalCount);
    String compact = "";

    for (int i = sent; i < batchEnd; i++) {
      String line    = lines[i];
      int comma1     = line.indexOf(',');
      int comma2     = line.indexOf(',', comma1 + 1);
      int comma3     = line.lastIndexOf(',');
      String ts      = line.substring(0, comma1);
      float truckV   = line.substring(comma1 + 1, comma2).toFloat();
      float battV    = line.substring(comma2 + 1, comma3).toFloat();
      String chargeS = line.substring(comma3 + 1);
      chargeS.trim();
      if (i > sent) compact += "|";
      compact += ts + "," + String(truckV, 3) + "," + String(battV, 3) +
                 "," + ((chargeS == "true") ? "1" : "0");
    }

    if (!sendToGoogleSheets(compact)) {
      debugLog("WiFi upload batch failed");
      return;
    }

    for (int i = sent; i < batchEnd; i++) archiveRecord(lines[i]);
    sent = batchEnd;
    feedWatchdog();
  }

  LittleFS.remove(PENDING_FILE);
  debugLog("WiFi upload complete");
}

// ===== OTA VIA WIFI =====
void checkAndApplyOTAWiFi() {
  debugLog("Checking OTA...");

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  http.begin(secureClient, VOLTMON_OTA_VERSION_URL);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  int httpCode = http.GET();
  feedWatchdog();

  if (httpCode != 200) {
    debugLog("OTA version check failed — HTTP " + String(httpCode));
    http.end();
    return;
  }

  String remoteVersion = http.getString();
  http.end();
  remoteVersion.trim();

  int localVer  = parseVersion(String(VOLTMON_VERSION));
  int remoteVer = parseVersion(remoteVersion);

  debugLog("Local: " + String(VOLTMON_VERSION) + " Remote: " + remoteVersion);

  if (remoteVer <= localVer) { debugLog("Firmware up to date"); return; }

  debugLog("Updating to " + remoteVersion + "...");
  esp_task_wdt_init(300, false);

  http.begin(secureClient, VOLTMON_OTA_FIRMWARE_URL);
  http.setTimeout(60000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  httpCode = http.GET();
  feedWatchdog();

  if (httpCode != 200) {
    debugLog("OTA download failed — HTTP " + String(httpCode));
    http.end();
    esp_task_wdt_init(120, false);
    return;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0 || !Update.begin(contentLength)) {
    debugLog("OTA begin failed");
    http.end();
    esp_task_wdt_init(120, false);
    return;
  }

  WiFiClient* stream  = http.getStreamPtr();
  size_t written      = 0;
  uint8_t buf[512];
  unsigned long lastFeed     = millis();
  unsigned long lastProgress = millis();

  while (http.connected() && written < (size_t)contentLength) {
    size_t available = stream->available();
    if (available) {
      size_t toRead    = min(available, sizeof(buf));
      size_t bytesRead = stream->readBytes(buf, toRead);
      written += Update.write(buf, bytesRead);
    }
    if (millis() - lastFeed > 5000)      { feedWatchdog(); lastFeed = millis(); }
    if (millis() - lastProgress > 10000) {
      debugLog("OTA: " + String(written * 100 / contentLength) + "%");
      lastProgress = millis();
    }
    delay(1);
  }
  http.end();

  if (written != (size_t)contentLength || !Update.end(true)) {
    debugLog("OTA failed");
    Update.abort();
    esp_task_wdt_init(120, false);
    return;
  }

  debugLog("OTA success — rebooting into " + remoteVersion);
  Serial.flush();
  delay(500);
  ESP.restart();
}

// ===== WIFI FALLBACK CYCLE =====
// Full cycle: upload pending + publish MQTT + OTA check
void doWiFiFallback(float truckVolts, float battVolts) {
  debugLog("WiFi fallback triggered — bleFailCount=" + String(bleFailCount));

  if (!connectWiFi()) {
    debugLog("WiFi fallback: no network available");
    if (portalActive) return;  // portal already running
    // Start portal if truck is running
    WiFi.mode(WIFI_AP_STA);
    delay(200);
    startPortal(deviceConfig, wifiCreds);
    return;
  }

  if (!ntpSynced) {
    if (syncNTP()) { ntpSynced = true; saveState(); }
  }

  uploadPendingViaWiFi();
  publishLatestReading(truckVolts, battVolts);
  checkAndApplyOTAWiFi();
  disconnectWiFi();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    lastWiFiAttempt = mktime(&timeinfo);
  }
  bleFailCount = 0;
  saveState();

  if (portalActive) stopPortal();
}

// ===== SHOULD TRY WIFI FALLBACK =====
bool shouldTryWiFi(bool truckRunning) {
  if (truckRunning) return true;        // always try when running
  if (bleFailCount >= 12) return true;  // once per hour when parked
  return false;
}

// ===== BLE TRANSFER =====
bool doBLETransfer() {
  debugLog("BLE: attempting transfer...");
  bleServerInit(deviceConfig);
  bool ok = bleScanAndTransfer(deviceConfig, hasPendingRecords());
  bleServerStop();

  if (ok) {
    LittleFS.remove(PENDING_FILE);
    recordCount  = 0;
    bleFailCount = 0;
    saveState();
    debugLog("BLE transfer successful");
  } else {
    bleFailCount++;
    saveState();
    debugLog("BLE transfer failed — bleFailCount=" + String(bleFailCount));
  }

  return ok;
}

// ===== GOTO SLEEP =====
void goToSleep() {
  if (portalActive) stopPortal();
  bleServerStop();
  digitalWrite(PIN_CHARGE_MOSFET, LOW);

  int sleepSecs = (deviceConfig.sleepSeconds > 0) ?
                   deviceConfig.sleepSeconds : SLEEP_SECONDS;

  debugLog("Sleeping " + String(sleepSecs) + "s");
  Serial.flush();
  delay(100);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSecs * 1000000ULL);
  esp_deep_sleep_start();
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(120, false);

  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_DEEPSLEEP) {
    Serial.println("Non-sleep reset — 10s flash window");
    delay(10000);
  } else {
    delay(5000);
  }

  Serial.print("Reset reason: ");
  Serial.println((int)reason);

  pinMode(PIN_TRUCK_SOURCE, OUTPUT);
  digitalWrite(PIN_TRUCK_SOURCE, LOW);
  pinMode(PIN_CHARGE_MOSFET, OUTPUT);
  digitalWrite(PIN_CHARGE_MOSFET, LOW);
  pinMode(PIN_TRUCK_ADC, INPUT);
  pinMode(PIN_BATT_ADC, INPUT);

  analogSetAttenuation(ADC_11db);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12,
                            ADC_WIDTH_BIT_12, 1100, &adc_chars);

  Wire.begin(PIN_DS3231_SDA, PIN_DS3231_SCL);
  if (!rtc.begin()) Serial.println("WARNING: DS3231 not found");

  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS failed");
    while (1) delay(100);
  }
  fsReady = true;

  File testFile = LittleFS.open("/fstest", "w");
  if (!testFile) {
    Serial.println("LittleFS not writable — reformatting...");
    fsReady = false;
    LittleFS.end();
    LittleFS.format();
    if (!LittleFS.begin(true)) {
      Serial.println("ERROR: LittleFS failed after format");
      while (1) delay(100);
    }
    fsReady = true;
  } else {
    testFile.close();
    LittleFS.remove("/fstest");
    Serial.println("LittleFS OK");
  }

  // Load config and credentials
  loadConfig(deviceConfig);
  loadWiFiCredentials(wifiCreds);

  // Seed from secrets.h on first flash (owner device only)
#ifdef SEED_FROM_SECRETS
  if (!deviceConfig.configured) {
    seedFromSecrets(deviceConfig, wifiCreds);
  }
#endif

  // Migration — populate LittleFS from compiled fallbacks if empty
  if (strlen(deviceConfig.deviceName) == 0) {
    debugLog("Migration: applying fallback config");
    strlcpy(deviceConfig.deviceName,  FALLBACK_DEVICE_NAME, sizeof(deviceConfig.deviceName));
    strlcpy(deviceConfig.aioUsername, AIO_USERNAME,         sizeof(deviceConfig.aioUsername));
    strlcpy(deviceConfig.aioKey,      AIO_KEY,              sizeof(deviceConfig.aioKey));
    deviceConfig.configured   = true;
    deviceConfig.sleepSeconds = SLEEP_SECONDS;
    deviceConfig.debugMode    = false;
    saveConfig(deviceConfig);
    debugLog("Migration: config saved — device: " + String(deviceConfig.deviceName));
  }

  if (wifiCreds.count == 0) {
    debugLog("Migration: applying fallback WiFi");
    for (int i = 0; i < WIFI_COUNT && i < MAX_WIFI_NETWORKS; i++) {
      wifiCreds.networks[i].ssid = String(WIFI_SSIDS[i]);
      wifiCreds.networks[i].pass = String(WIFI_PASSWORDS[i]);
      wifiCreds.count++;
    }
    saveWiFiCredentials(wifiCreds);
    debugLog("Migration: WiFi saved — " + String(wifiCreds.count) + " networks");
  }

  loadState();
  setupMQTT();

  DateTime rtcCheck = rtc.now();
  if (rtcCheck.year() < 2020) {
    debugLog("RTC invalid — will sync on next WiFi connection");
    ntpSynced = false;
    saveState();
  }

  wakeCount++;
  saveState();

  debugLog("=== Wake #" + String(wakeCount) +
           " [" + String(deviceConfig.deviceName) + "] v" + String(VOLTMON_VERSION) +
           " reset=" + String((int)reason) +
           (deviceConfig.debugMode ? " DEBUG" : "") + " ===");

  if (wakeCount == 1) {
    dumpDebugLog();
    dumpPendingFile();
  }

  // ===== NOT CONFIGURED — start portal =====
  if (!deviceConfig.configured) {
    debugLog("Not configured — starting portal");
    WiFi.mode(WIFI_AP_STA);
    delay(200);
    startPortal(deviceConfig, wifiCreds);

    while (!deviceConfig.configured) {
      portalLoop();
      if (credsUpdated || configUpdated) {
        credsUpdated  = false;
        configUpdated = false;
        loadConfig(deviceConfig);
        loadWiFiCredentials(wifiCreds);
        setupMQTT();
        if (deviceConfig.configured) {
          stopPortal();
          break;
        }
      }
      delay(500);
      feedWatchdog();
    }
  }

  // Read voltages
  DateTime now      = rtc.now();
  float rawTruck    = readVoltage(PIN_TRUCK_SOURCE, PIN_TRUCK_ADC, TRUCK_DIVIDER_RATIO);
  float truckVolts  = applyTruckCalibration(rawTruck);
  float battVolts   = readBattVoltage();
  bool truckRunning = (truckVolts > VOLTAGE_RUNNING);
  bool charging     = truckRunning && (battVolts < BATT_START_CHARGE);

  debugLog(getTimestamp(now) +
           " | Truck: " + String(truckVolts, 3) +
           "V | Batt: " + String(battVolts, 3) +
           "V | Running: " + (truckRunning ? "YES" : "NO") +
           " | Charging: " + (charging ? "YES" : "NO"));

  // Write reading
  writePending(now, truckVolts, battVolts, charging);
  recordCount++;
  saveState();

  // ===== BLE TRANSFER ATTEMPT =====
  bool bleSuccess = false;
  if (hasPendingRecords()) {
    bleSuccess = doBLETransfer();
  }

  // ===== WIFI FALLBACK =====
  if (!bleSuccess && shouldTryWiFi(truckRunning)) {
    doWiFiFallback(truckVolts, battVolts);
  }

  // ===== TRUCK RUNNING LOOP =====
  if (truckRunning) {
    digitalWrite(PIN_CHARGE_MOSFET, charging ? HIGH : LOW);

    while (truckRunning) {
      portalLoop();

      // Check portal updates
      if (portalActive && (credsUpdated || configUpdated)) {
        credsUpdated  = false;
        configUpdated = false;
        loadConfig(deviceConfig);
        loadWiFiCredentials(wifiCreds);
        setupMQTT();
        if (connectWiFi()) {
          stopPortal();
          if (!ntpSynced && syncNTP()) { ntpSynced = true; saveState(); }
          uploadPendingViaWiFi();
          disconnectWiFi();
          bleFailCount = 0;
          saveState();
        }
      }

      for (int i = 0; i < CHARGE_CHECK_SECONDS; i++) {
        delay(1000);
        feedWatchdog();
      }

      rawTruck     = readVoltage(PIN_TRUCK_SOURCE, PIN_TRUCK_ADC, TRUCK_DIVIDER_RATIO);
      truckVolts   = applyTruckCalibration(rawTruck);
      battVolts    = readBattVoltage();
      truckRunning = (truckVolts > VOLTAGE_RUNNING);

      if (charging && battVolts > BATT_STOP_CHARGE) {
        charging = false;
        debugLog("Charging stopped — " + String(battVolts, 3) + "V");
      } else if (!charging && truckRunning && battVolts < BATT_START_CHARGE) {
        charging = true;
        debugLog("Charging started — " + String(battVolts, 3) + "V");
      }

      digitalWrite(PIN_CHARGE_MOSFET, charging ? HIGH : LOW);
      writePending(rtc.now(), truckVolts, battVolts, charging);
      recordCount++;
      saveState();

      if (recordCount >= UPLOAD_EVERY) {
        bool ok = doBLETransfer();
        if (!ok) {
          doWiFiFallback(truckVolts, battVolts);
        } else {
          recordCount = 0;
          saveState();
        }
      }
    }

    debugLog("Truck stopped");
  }

  goToSleep();
}

void loop() {
  // Never runs
}