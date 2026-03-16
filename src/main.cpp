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

// State
int  recordCount     = 0;
bool ntpSynced       = false;
int  wakeCount       = 0;
int  failedScanCount = 0;

esp_adc_cal_characteristics_t adc_chars;
RTC_DS3231 rtc;

WiFiClient mqttWifiClient;
Adafruit_MQTT_Client mqtt(&mqttWifiClient, AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY);
char TOPIC_TRUCK[64];
char TOPIC_BATT[64];
char TOPIC_DEBUG[64];
Adafruit_MQTT_Publish* feedTruckVoltage;
Adafruit_MQTT_Publish* feedBattVoltage;
Adafruit_MQTT_Publish* feedDebug;

bool fsReady = false;

// ===== WATCHDOG =====
void feedWatchdog() {
  esp_task_wdt_reset();
}

// ===== VERSION PARSING =====
int parseVersion(String v) {
  v.trim();
  int major = 0, minor = 0, patch = 0;
  int first = v.indexOf('.');
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
    else if (line.startsWith("failedScanCount="))
      failedScanCount = line.substring(16).toInt();
  }
  f.close();

  debugLog("State loaded — wakes=" + String(wakeCount) +
           " records=" + String(recordCount) +
           " ntp=" + String(ntpSynced) +
           " failedScans=" + String(failedScanCount));
}

void saveState() {
  if (!fsReady) return;

  File f = LittleFS.open(STATE_FILE, "w");
  if (!f) {
    debugLog("ERROR: Could not save state");
    return;
  }
  f.println("recordCount=" + String(recordCount));
  f.println("wakeCount="   + String(wakeCount));
  f.println("ntpSynced="   + String(ntpSynced ? 1 : 0));
  f.println("failedScanCount=" + String(failedScanCount));
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
    return TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-1][1] + slope * (rawVolts - TRUCK_CAL_TABLE[TRUCK_CAL_TABLE_SIZE-1][0]);
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
    return BATT_CAL_TABLE[BATT_CAL_TABLE_SIZE-1][1] + slope * (rawVolts - BATT_CAL_TABLE[BATT_CAL_TABLE_SIZE-1][0]);

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
  uint32_t raw = sum / ADC_SAMPLES;
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
  uint32_t raw = sum / ADC_SAMPLES;
  uint32_t millivolts = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
  float rawVolts = (millivolts / 1000.0) * BATT_DIVIDER_RATIO;
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
    debugLog("ERROR: Could not open pending file after retries");
    Serial.println("LittleFS total: " + String(LittleFS.totalBytes()) +
                   " used: " + String(LittleFS.usedBytes()));
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

// ===== WIFI SCAN =====
// Scans for known networks, returns index into WIFI_SSIDS of strongest found
// Populates bestBSSID and bestChannel for use in connection
// Returns -1 if no known network found
int scanForKnownNetworks(uint8_t* bestBSSID, int* bestChannel) {
  debugLog("Scanning for known networks...");

  int found = WiFi.scanNetworks();

  if (found <= 0) {
    debugLog("No networks found");
    WiFi.scanDelete();
    return -1;
  }

  debugLog("Networks found: " + String(found));

  int bestIndex = -1;
  int bestRSSI  = -999;

  for (int i = 0; i < found; i++) {
    String scannedSSID = WiFi.SSID(i);
    int    scannedRSSI = WiFi.RSSI(i);
    int    scannedCh   = WiFi.channel(i);
    debugLog("  " + scannedSSID + " (" + String(scannedRSSI) + " dBm) ch" + String(scannedCh));

    for (int n = 0; n < WIFI_COUNT; n++) {
      if (scannedSSID == String(WIFI_SSIDS[n])) {
        if (scannedRSSI > bestRSSI) {
          bestRSSI     = scannedRSSI;
          bestIndex    = n;
          *bestChannel = scannedCh;
          memcpy(bestBSSID, WiFi.BSSID(i), 6);
        }
      }
    }
  }

  WiFi.scanDelete();

  if (bestIndex >= 0) {
    debugLog("Best known network: " + String(WIFI_SSIDS[bestIndex]) +
             " (" + String(bestRSSI) + " dBm) ch" + String(*bestChannel));
  } else {
    debugLog("No known networks in range");
  }

  return bestIndex;
}

// ===== WIFI CONNECT =====
bool connectWiFi() {
  debugLog("Connecting to WiFi...");

  // Scan first as a gate check — no point connecting if network not visible
  WiFi.mode(WIFI_STA);
  delay(500);

  uint8_t bestBSSID[6];
  int     bestChannel  = 0;
  int     networkIndex = scanForKnownNetworks(bestBSSID, &bestChannel);

  if (networkIndex < 0) {
    failedScanCount++;
    saveState();
    debugLog("No known network in range — failedScans=" + String(failedScanCount));
    return false;
  }

  // Full radio reset after scan before connecting — same sequence that worked before scan was added
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  delay(1000);

  for (int attempt = 1; attempt <= WIFI_MAX_ATTEMPTS; attempt++) {
    debugLog("Attempt " + String(attempt) + " of " + String(WIFI_MAX_ATTEMPTS));

    WiFi.begin(WIFI_SSIDS[networkIndex], WIFI_PASSWORDS[networkIndex]);

    int dots = 0;
    while (WiFi.status() != WL_CONNECTED && dots < 20) {
      delay(500);
      feedWatchdog();
      dots++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      debugLog("WiFi connected: " + WiFi.localIP().toString() +
               " RSSI: " + String(WiFi.RSSI()) + " dBm" +
               " ch" + String(bestChannel));
      return true;
    }

    debugLog("Connect failed — status: " + String(WiFi.status()));
    if (attempt < WIFI_MAX_ATTEMPTS) {
      WiFi.disconnect(false);
      delay(1000);
      debugLog("Waiting before retry...");
      delay(2000);
    }
  }

  failedScanCount++;
  saveState();
  debugLog("All attempts failed — failedScans=" + String(failedScanCount));
  return false;
}

void disconnectWiFi() {
  mqtt.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
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

  debugLog("RTC synced to NTP: " + getTimestamp(rtc.now()));
  return true;
}

// ===== MQTT =====
bool connectMQTT() {
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
  debugLog(ok1 ? "MQTT truck published: " + String(truckVolts, 3) + "V"
               : "MQTT truck publish failed");
  debugLog(ok2 ? "MQTT batt published: " + String(battVolts, 3) + "V"
               : "MQTT batt publish failed");
  mqtt.disconnect();
}

// ===== GOOGLE SHEETS =====
bool sendToGoogleSheets(String compactData) {
  debugLog("Sending to Google Sheets...");

  compactData.replace(" ", "%20");
  debugLog("Payload length: " + String(compactData.length()) + " chars");

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  String url = String(GS_SCRIPT_URL) + "?d=" + compactData;
  debugLog("URL length: " + String(url.length()));

  HTTPClient http;
  http.begin(secureClient, url);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  int httpCode = http.GET();
  feedWatchdog();

  String response = http.getString();
  http.end();

  debugLog("HTTP code: " + String(httpCode));
  debugLog("Response: " + response.substring(0, 100));

  return response.indexOf("\"ok\"") >= 0;
}

// ===== UPLOAD PENDING =====
void uploadPending() {
  if (!fsReady || !LittleFS.exists(PENDING_FILE)) {
    debugLog("No pending records to upload");
    return;
  }

  File f = LittleFS.open(PENDING_FILE, "r");
  if (!f) {
    debugLog("ERROR: Could not open pending file");
    return;
  }

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

  if (totalCount == 0) {
    debugLog("No valid records to upload");
    return;
  }

  debugLog("Total records to upload: " + String(totalCount));

  const int BATCH_SIZE = 40;
  int sent = 0;

  while (sent < totalCount) {
    feedWatchdog();

    int batchEnd = min(sent + BATCH_SIZE, totalCount);
    String compact = "";

    for (int i = sent; i < batchEnd; i++) {
      String line = lines[i];
      int comma1 = line.indexOf(',');
      int comma2 = line.indexOf(',', comma1 + 1);
      int comma3 = line.lastIndexOf(',');

      String ts      = line.substring(0, comma1);
      float truckV   = line.substring(comma1 + 1, comma2).toFloat();
      float battV    = line.substring(comma2 + 1, comma3).toFloat();
      String chargeS = line.substring(comma3 + 1);
      chargeS.trim();
      String chargeCompact = (chargeS == "true") ? "1" : "0";

      if (i > sent) compact += "|";
      compact += ts + "," + String(truckV, 3) + "," + String(battV, 3) + "," + chargeCompact;
    }

    debugLog("Sending batch " + String(sent/BATCH_SIZE + 1) +
             ": records " + String(sent+1) + "-" + String(batchEnd) +
             " (" + String(compact.length()) + " chars)");

    if (!sendToGoogleSheets(compact)) {
      debugLog("Batch failed — aborting upload, keeping pending for retry");
      return;
    }

    for (int i = sent; i < batchEnd; i++) {
      archiveRecord(lines[i]);
    }

    sent = batchEnd;
    feedWatchdog();
  }

  LittleFS.remove(PENDING_FILE);
  debugLog("Upload complete — " + String(totalCount) + " records, pending cleared");
}

// ===== OTA =====
void checkAndApplyOTA() {
  debugLog("Checking for OTA update...");

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  http.begin(secureClient, OTA_VERSION_URL);
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

  int localVer  = parseVersion(String(FIRMWARE_VERSION));
  int remoteVer = parseVersion(remoteVersion);

  debugLog("Local: " + String(FIRMWARE_VERSION) + " (" + String(localVer) + ")" +
           " Remote: " + remoteVersion + " (" + String(remoteVer) + ")");

  if (remoteVer <= localVer) {
    debugLog("Firmware up to date");
    return;
  }

  debugLog("New firmware available: " + remoteVersion + " — downloading...");

  esp_task_wdt_init(300, false);

  http.begin(secureClient, OTA_FIRMWARE_URL);
  http.setTimeout(60000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  httpCode = http.GET();
  feedWatchdog();

  if (httpCode != 200) {
    debugLog("OTA firmware download failed — HTTP " + String(httpCode));
    http.end();
    esp_task_wdt_init(120, false);
    if (connectMQTT()) {
      String status = "500_" + remoteVersion;
      Adafruit_MQTT_Publish feedOTA(&mqtt, AIO_USERNAME "/feeds/charge-status");
      feedOTA.publish(status.c_str());
      mqtt.disconnect();
    }
    return;
  }

  int contentLength = http.getSize();
  debugLog("Firmware size: " + String(contentLength) + " bytes");

  if (contentLength <= 0) {
    debugLog("OTA invalid content length");
    http.end();
    esp_task_wdt_init(120, false);
    return;
  }

  if (!Update.begin(contentLength)) {
    debugLog("OTA Update.begin failed — not enough space");
    http.end();
    esp_task_wdt_init(120, false);
    return;
  }

  WiFiClient* stream = http.getStreamPtr();
  size_t written = 0;
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

    if (millis() - lastFeed > 5000) {
      feedWatchdog();
      lastFeed = millis();
    }

    if (millis() - lastProgress > 10000) {
      debugLog("OTA progress: " + String(written) + "/" + String(contentLength) +
               " (" + String(written * 100 / contentLength) + "%)");
      lastProgress = millis();
    }

    delay(1);
  }
  http.end();

  if (written != (size_t)contentLength) {
    debugLog("OTA write incomplete: " + String(written) + "/" + String(contentLength));
    Update.abort();
    esp_task_wdt_init(120, false);
    return;
  }

  if (!Update.end(true)) {
    debugLog("OTA Update.end failed — error: " + String(Update.getError()));
    esp_task_wdt_init(120, false);
    if (connectMQTT()) {
      String status = "500_" + remoteVersion;
      Adafruit_MQTT_Publish feedOTA(&mqtt, AIO_USERNAME "/feeds/charge-status");
      feedOTA.publish(status.c_str());
      mqtt.disconnect();
    }
    return;
  }

  debugLog("OTA success — new version: " + remoteVersion);

  if (connectMQTT()) {
    String status = "200_" + remoteVersion;
    Adafruit_MQTT_Publish feedOTA(&mqtt, AIO_USERNAME "/feeds/charge-status");
    feedOTA.publish(status.c_str());
    debugLog("OTA status published: " + status);
    mqtt.disconnect();
  }

  debugLog("Rebooting into new firmware...");
  Serial.flush();
  delay(500);
  ESP.restart();
}

// ===== DEBUG CYCLE =====
void doDebugCycle(float truckVolts, float battVolts) {
  debugLog("=== DEBUG CYCLE ===");

  if (!connectWiFi()) {
    debugLog("No network found — failedScans=" + String(failedScanCount));
    return;
  }

  if (!ntpSynced) {
    if (syncNTP()) {
      ntpSynced = true;
      saveState();
    }
  }

  if (connectMQTT()) {
    uint8_t* bssid = WiFi.BSSID();
        char bssidStr[18];
        snprintf(bssidStr, sizeof(bssidStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);

        String msg = "RSSI:" + String(WiFi.RSSI()) +
                    " Ch:" + String(WiFi.channel()) +
                    " AP:" + String(bssidStr) +
                    " Truck:" + String(truckVolts, 3) +
                    " Batt:" + String(battVolts, 3) +
                    " FailedScans:" + String(failedScanCount);

        bool ok = feedDebug->publish(msg.c_str());
        debugLog(ok ? "Debug published: " + msg : "Debug publish failed");
        mqtt.disconnect();
  }

  failedScanCount = 0;
  saveState();

  checkAndApplyOTA();
  disconnectWiFi();
}

// ===== UPLOAD CYCLE =====
bool doUploadCycle(float truckVolts, float battVolts) {
  debugLog("Upload cycle triggered — " + String(recordCount) + " records");

  if (connectWiFi()) {
    if (!ntpSynced) {
      if (syncNTP()) {
        ntpSynced = true;
        saveState();
      }
    }
    uploadPending();
    publishLatestReading(truckVolts, battVolts);
    checkAndApplyOTA();
    disconnectWiFi();

    failedScanCount = 0;
    saveState();
    return true;
  }

  debugLog("WiFi unavailable — retry next cycle");
  return false;
}

// ===== GOTO SLEEP =====
void goToSleep() {
  digitalWrite(PIN_CHARGE_MOSFET, LOW);
  debugLog("Sleeping " + String(SLEEP_SECONDS) + "s | wakes=" +
           String(wakeCount) + " records=" + String(recordCount));
  Serial.flush();
  delay(100);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  // Watchdog FIRST — before any delays
  esp_task_wdt_init(120, false);

  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_DEEPSLEEP) {
    Serial.println("Non-sleep reset — 10 second flash window");
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
  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_12,
    ADC_WIDTH_BIT_12,
    1100,
    &adc_chars
  );

  Wire.begin(PIN_DS3231_SDA, PIN_DS3231_SCL);
  if (!rtc.begin()) {
    Serial.println("WARNING: DS3231 not found — using default timestamp");
  }

  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS failed even with format on fail");
    while (1) delay(100);
  }
  fsReady = true;

  File testFile = LittleFS.open("/fstest", "w");
  if (!testFile) {
    Serial.println("LittleFS not writable — forcing format...");
    fsReady = false;
    LittleFS.end();
    LittleFS.format();
    if (!LittleFS.begin(true)) {
      Serial.println("ERROR: LittleFS failed after forced format");
      while (1) delay(100);
    }
    fsReady = true;
  } else {
    testFile.close();
    LittleFS.remove("/fstest");
    Serial.println("LittleFS writable — OK");
  }

  loadState();

  // Force NTP sync if RTC time is invalid
  DateTime rtcCheck = rtc.now();
  if (rtcCheck.year() < 2020) {
    debugLog("RTC time invalid — forcing NTP sync");
    ntpSynced = false;
    saveState();
  }

  snprintf(TOPIC_TRUCK, sizeof(TOPIC_TRUCK), "%s/feeds/truck-voltage",  AIO_USERNAME);
  snprintf(TOPIC_BATT,  sizeof(TOPIC_BATT),  "%s/feeds/device-voltage", AIO_USERNAME);
  snprintf(TOPIC_DEBUG, sizeof(TOPIC_DEBUG), "%s/feeds/debug-messages", AIO_USERNAME);
  feedTruckVoltage = new Adafruit_MQTT_Publish(&mqtt, TOPIC_TRUCK);
  feedBattVoltage  = new Adafruit_MQTT_Publish(&mqtt, TOPIC_BATT);
  feedDebug        = new Adafruit_MQTT_Publish(&mqtt, TOPIC_DEBUG);

  wakeCount++;
  saveState();

  debugLog("=== Wake #" + String(wakeCount) +
           " reset=" + String((int)reason) +
           (DEBUG_MODE ? " DEBUG" : "") + " ===");

  if (wakeCount == 1) {
    dumpDebugLog();
    dumpPendingFile();
  }

  DateTime now      = rtc.now();
  float rawTruck    = readVoltage(PIN_TRUCK_SOURCE, PIN_TRUCK_ADC, TRUCK_DIVIDER_RATIO);
  float truckVolts  = applyTruckCalibration(rawTruck);
  float battVolts   = readBattVoltage();
  bool truckRunning = (truckVolts > VOLTAGE_RUNNING);
  bool charging     = truckRunning && (battVolts < BATT_START_CHARGE);

  debugLog(getTimestamp(now) +
           " | Raw: "      + String(rawTruck, 3) +
           "V | Truck: "   + String(truckVolts, 3) +
           "V | Batt: "    + String(battVolts, 3) +
           "V | Running: " + (truckRunning ? "YES" : "NO") +
           " | Charging: " + (charging ? "YES" : "NO"));

  // ===== DEBUG MODE =====
  if (DEBUG_MODE) {
    doDebugCycle(truckVolts, battVolts);
    debugLog("Debug loop — restarting in " + String(DEBUG_SLEEP_SECONDS) + "s");
    Serial.flush();
    delay(DEBUG_SLEEP_SECONDS * 1000);
    esp_task_wdt_deinit();
    ESP.restart();
  }

  // ===== NORMAL OPERATION =====
  digitalWrite(PIN_CHARGE_MOSFET, charging ? HIGH : LOW);
  writePending(now, truckVolts, battVolts, charging);

  recordCount++;
  saveState();

  debugLog("record " + String(recordCount) + "/" + String(UPLOAD_EVERY));

  while (truckRunning) {
    debugLog("Truck running — Charging: " + String(charging ? "YES" : "NO") +
             " | Truck: " + String(truckVolts, 3) +
             "V | Batt: " + String(battVolts, 3) + "V");

    for (int i = 0; i < CHARGE_CHECK_SECONDS; i++) {
      delay(1000);
      feedWatchdog();
    }

    rawTruck     = readVoltage(PIN_TRUCK_SOURCE, PIN_TRUCK_ADC, TRUCK_DIVIDER_RATIO);
    truckVolts   = applyTruckCalibration(rawTruck);
    battVolts    = readBattVoltage();
    truckRunning = (truckVolts > VOLTAGE_RUNNING);

    if (charging && (battVolts > BATT_STOP_CHARGE)) {
      charging = false;
      debugLog("Charging stopped — batt at " + String(battVolts, 3) + "V");
    } else if (!charging && truckRunning && battVolts < BATT_START_CHARGE) {
      charging = true;
      debugLog("Charging started — batt at " + String(battVolts, 3) + "V");
    }

    digitalWrite(PIN_CHARGE_MOSFET, charging ? HIGH : LOW);

    writePending(rtc.now(), truckVolts, battVolts, charging);
    recordCount++;
    saveState();

    if (recordCount >= UPLOAD_EVERY) {
      if (doUploadCycle(truckVolts, battVolts)) {
        recordCount = 0;
        saveState();
      }
    }
  }

  debugLog("Truck stopped — proceeding to upload check");

  if (recordCount >= UPLOAD_EVERY) {
    if (doUploadCycle(truckVolts, battVolts)) {
      recordCount = 0;
      saveState();
    }
  }

  goToSleep();
}

void loop() {
  // Never runs
}