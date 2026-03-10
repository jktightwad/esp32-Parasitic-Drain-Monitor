#include "Arduino.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
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
int  recordCount  = 0;
bool ntpSynced    = false;
int  wakeCount    = 0;

esp_adc_cal_characteristics_t adc_chars;
RTC_DS3231 rtc;

WiFiClient mqttWifiClient;
Adafruit_MQTT_Client mqtt(&mqttWifiClient, AIO_SERVER, AIO_PORT, AIO_USERNAME, AIO_KEY);
char TOPIC_TRUCK[64];
char TOPIC_BATT[64];
Adafruit_MQTT_Publish* feedTruckVoltage;
Adafruit_MQTT_Publish* feedBattVoltage;

bool fsReady = false;

// ===== WATCHDOG =====
void feedWatchdog() {
  esp_task_wdt_reset();
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
  }
  f.close();

  debugLog("State loaded — wakes=" + String(wakeCount) +
           " records=" + String(recordCount) +
           " ntp=" + String(ntpSynced));
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

  File f = LittleFS.open(PENDING_FILE, "a");
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

// ===== WIFI =====
bool connectWiFi() {
  debugLog("Connecting to WiFi...");

  WiFi.disconnect();
  delay(500);
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(500);

  // Try each network in order
  for (int n = 0; n < WIFI_COUNT; n++) {
    debugLog("Trying network: " + String(WIFI_SSIDS[n]));
    WiFi.begin(WIFI_SSIDS[n], WIFI_PASSWORDS[n]);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      feedWatchdog();
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      debugLog("WiFi connected: " + WiFi.localIP().toString());
      return true;
    }

    debugLog("Network " + String(WIFI_SSIDS[n]) + " failed");
    WiFi.disconnect();
    delay(500);
  }

  debugLog("All networks failed");
  return false;
}

void disconnectWiFi() {
  mqtt.disconnect();
  WiFi.disconnect();
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

    int batchCount = batchEnd - sent;
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

// ===== UPLOAD CYCLE =====
bool doUploadCycle(float truckVolts, float battVolts) {
  debugLog("Upload cycle triggered — " + String(recordCount) + " records");
  if (connectWiFi()) {
    if (!ntpSynced) {
      if (syncNTP()) ntpSynced = true;
    }
    uploadPending();
    publishLatestReading(truckVolts, battVolts);
    disconnectWiFi();
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

  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_DEEPSLEEP) {
    Serial.println("Non-sleep reset — 10 second flash window");
    delay(10000);
  } else {
    delay(5000);
  }

  esp_task_wdt_init(120, false);

  Serial.print("Reset reason: ");
  Serial.println((int)reason);

  pinMode(PIN_TRUCK_SOURCE, OUTPUT);
  digitalWrite(PIN_TRUCK_SOURCE, LOW);
  pinMode(PIN_CHARGE_MOSFET, OUTPUT);
  digitalWrite(PIN_CHARGE_MOSFET, LOW);

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
    Serial.println("ERROR: DS3231 not found");
    while (1) delay(100);
  }

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed — formatting...");
    LittleFS.format();
    if (!LittleFS.begin(false)) {
      Serial.println("ERROR: LittleFS failed after format");
      while (1) delay(100);
    }
  }
  fsReady = true;

  // Verify filesystem is writable
  File testFile = LittleFS.open("/fstest", "w");
  if (!testFile) {
    Serial.println("LittleFS not writable — reformatting...");
    LittleFS.end();
    LittleFS.format();
    if (!LittleFS.begin(false)) {
      Serial.println("ERROR: LittleFS failed after reformat");
      while (1) delay(100);
    }
    Serial.println("LittleFS reformatted successfully");
  } else {
    testFile.close();
    LittleFS.remove("/fstest");
  }

  loadState();

  snprintf(TOPIC_TRUCK, sizeof(TOPIC_TRUCK), "%s/feeds/truck-voltage", AIO_USERNAME);
  snprintf(TOPIC_BATT,  sizeof(TOPIC_BATT),  "%s/feeds/device-voltage", AIO_USERNAME);
  feedTruckVoltage = new Adafruit_MQTT_Publish(&mqtt, TOPIC_TRUCK);
  feedBattVoltage  = new Adafruit_MQTT_Publish(&mqtt, TOPIC_BATT);

  wakeCount++;
  recordCount++;
  saveState();

  debugLog("=== Wake #" + String(wakeCount) +
           " record " + String(recordCount) +
           "/" + String(UPLOAD_EVERY) +
           " reset=" + String((int)reason) + " ===");

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
           " | Raw: "       + String(rawTruck, 3) +
           "V | Truck: "    + String(truckVolts, 3) +
           "V | Batt: "     + String(battVolts, 3) +
           "V | Running: "  + (truckRunning ? "YES" : "NO") +
           " | Charging: "  + (charging ? "YES" : "NO"));

  digitalWrite(PIN_CHARGE_MOSFET, charging ? HIGH : LOW);
  writePending(now, truckVolts, battVolts, charging);

  while (truckRunning) {
    debugLog("Truck running — Charging: " + String(charging ? "YES" : "NO") +
             " | Truck: "  + String(truckVolts, 3) +
             "V | Batt: "  + String(battVolts, 3) + "V");

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