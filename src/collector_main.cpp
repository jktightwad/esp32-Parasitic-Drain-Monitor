#include "Arduino.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "esp_task_wdt.h"
#include "config.h"
#include "storage.h"
#include "ble_client.h"

// ===== NEOPIXEL =====
Adafruit_NeoPixel strip(VM_NEOPIXEL_COUNT, VM_PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ===== COLOR DEFINITIONS =====
#define COLOR_OFF       strip.Color(0,   0,   0)
#define COLOR_GREEN     strip.Color(0,   200, 0)
#define COLOR_RED       strip.Color(200, 0,   0)
#define COLOR_YELLOW    strip.Color(200, 180, 0)
#define COLOR_BLUE      strip.Color(0,   0,   200)
#define COLOR_CYAN      strip.Color(0,   200, 200)
#define COLOR_WHITE     strip.Color(200, 200, 200)
#define COLOR_PURPLE    strip.Color(150, 0,   200)
#define COLOR_DIM_BLUE  strip.Color(0,   0,   30)

// ===== LED HELPERS =====
void setWiFiLED(uint32_t color) {
  for (int i = LED_WIFI_START; i < LED_WIFI_START + LED_WIFI_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void setUploadLED(uint32_t color) {
  for (int i = LED_UPLOAD_START; i < LED_UPLOAD_START + LED_UPLOAD_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void setActivityLED(uint32_t color) {
  for (int i = LED_ACTIVITY_START; i < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void clearActivity() {
  setActivityLED(COLOR_OFF);
}

// Chase animation on activity LEDs
void animateChase(uint32_t color, int delayMs = 50) {
  for (int i = 0; i < LED_ACTIVITY_COUNT; i++) {
    for (int j = LED_ACTIVITY_START; j < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; j++) {
      strip.setPixelColor(j, COLOR_OFF);
    }
    strip.setPixelColor(LED_ACTIVITY_START + i, color);
    strip.show();
    delay(delayMs);
  }
}

// Pulse animation on activity LEDs
void animatePulse(uint32_t color, int steps = 20, int delayMs = 30) {
  // Fade in
  for (int i = 0; i <= steps; i++) {
    uint8_t r = ((color >> 16) & 0xFF) * i / steps;
    uint8_t g = ((color >> 8)  & 0xFF) * i / steps;
    uint8_t b = ((color)       & 0xFF) * i / steps;
    uint32_t c = strip.Color(r, g, b);
    setActivityLED(c);
    delay(delayMs);
  }
  // Fade out
  for (int i = steps; i >= 0; i--) {
    uint8_t r = ((color >> 16) & 0xFF) * i / steps;
    uint8_t g = ((color >> 8)  & 0xFF) * i / steps;
    uint8_t b = ((color)       & 0xFF) * i / steps;
    uint32_t c = strip.Color(r, g, b);
    setActivityLED(c);
    delay(delayMs);
  }
}

// Flash activity LEDs
void flashActivity(uint32_t color, int times = 3, int delayMs = 150) {
  for (int i = 0; i < times; i++) {
    setActivityLED(color);
    delay(delayMs);
    setActivityLED(COLOR_OFF);
    delay(delayMs);
  }
}

// ===== GLOBALS =====
bool fsReady   = false;
bool mqttReady = false;
bool wifiConnected = false;

WiFiClient           mqttWifiClient;
Adafruit_MQTT_Client mqtt(&mqttWifiClient, AIO_SERVER, AIO_PORT, "", "");

char TOPIC_TRUCK[80];
char TOPIC_BATT[80];
char TOPIC_DEBUG[80];
char TOPIC_CONTROL[80];

Adafruit_MQTT_Publish*   feedTruckVoltage = nullptr;
Adafruit_MQTT_Publish*   feedBattVoltage  = nullptr;
Adafruit_MQTT_Publish*   feedDebug        = nullptr;
Adafruit_MQTT_Subscribe* feedControl      = nullptr;

// Collector pending buffer file
#define COLLECTOR_PENDING_FILE  "/col_pending.csv"

// ===== WATCHDOG =====
void feedWatchdog() { esp_task_wdt_reset(); }

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

// ===== LITTLEFS =====
bool initFS() {
  if (!LittleFS.begin(true)) return false;
  File f = LittleFS.open("/fstest", "w");
  if (!f) {
    LittleFS.end();
    LittleFS.format();
    if (!LittleFS.begin(true)) return false;
  } else {
    f.close();
    LittleFS.remove("/fstest");
  }
  return true;
}

// ===== MQTT SETUP =====
void setupMQTT() {
  mqtt.~Adafruit_MQTT_Client();
  new (&mqtt) Adafruit_MQTT_Client(&mqttWifiClient, AIO_SERVER, AIO_PORT,
                                    AIO_USERNAME, AIO_KEY);

  snprintf(TOPIC_TRUCK,   sizeof(TOPIC_TRUCK),   "%s/feeds/truck-voltage",  AIO_USERNAME);
  snprintf(TOPIC_BATT,    sizeof(TOPIC_BATT),    "%s/feeds/device-voltage", AIO_USERNAME);
  snprintf(TOPIC_DEBUG,   sizeof(TOPIC_DEBUG),   "%s/feeds/debug-messages", AIO_USERNAME);
  snprintf(TOPIC_CONTROL, sizeof(TOPIC_CONTROL), "%s/feeds/voltmon-control", AIO_USERNAME);

  delete feedTruckVoltage;
  delete feedBattVoltage;
  delete feedDebug;
  delete feedControl;

  feedTruckVoltage = new Adafruit_MQTT_Publish(&mqtt,   TOPIC_TRUCK);
  feedBattVoltage  = new Adafruit_MQTT_Publish(&mqtt,   TOPIC_BATT);
  feedDebug        = new Adafruit_MQTT_Publish(&mqtt,   TOPIC_DEBUG);
  feedControl      = new Adafruit_MQTT_Subscribe(&mqtt, TOPIC_CONTROL);

  mqtt.subscribe(feedControl);
  mqttReady = true;
}

// ===== WIFI =====
bool connectWiFi() {
  Serial.println("WiFi connecting...");
  setWiFiLED(COLOR_YELLOW);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);

  Serial.println("WiFi SSID: [" + String(WIFI_SSIDS[0]) + "]");
  Serial.println("WiFi PASS length: " + String(strlen(WIFI_PASSWORDS[0])));
  WiFi.begin(WIFI_SSIDS[0], WIFI_PASSWORDS[0]);


  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    feedWatchdog();
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
    setWiFiLED(COLOR_GREEN);
    wifiConnected = true;
    return true;
  }

  Serial.println("WiFi failed");
  setWiFiLED(COLOR_RED);
  wifiConnected = false;
  return false;
}

void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    setWiFiLED(COLOR_YELLOW);
    Serial.println("WiFi lost — reconnecting...");
    WiFi.disconnect();
    delay(1000);
    connectWiFi();
  }
}

// ===== MQTT CONNECT =====
bool connectMQTT() {
  if (!wifiConnected) return false;

  int attempts = 0;
  while (mqtt.connect() != 0 && attempts < 3) {
    mqtt.disconnect();
    feedWatchdog();
    delay(2000);
    attempts++;
  }

  return mqtt.connected();
}

// ===== BUFFER RECORDS TO COLLECTOR PENDING =====
void bufferRecords(const String& deviceId, const String& records) {
  // Parse pipe-delimited records and prefix each with device ID
  // Format stored: deviceId,timestamp,truckV,battV,charging
  File f = LittleFS.open(COLLECTOR_PENDING_FILE, "a", true);  // true = create if not exists
  if (!f) {
    Serial.println("ERROR: Cannot open collector pending");
    return;
  }

  // Records arrive as: ts,truckV,battV,charging|ts,truckV,battV,charging|...
  int start = 0;
  while (start < (int)records.length()) {
    int end = records.indexOf('|', start);
    if (end < 0) end = records.length();
    String record = records.substring(start, end);
    record.trim();
    if (record.length() > 0) {
      f.println(deviceId + "," + record);
    }
    start = end + 1;
  }

  f.close();
  Serial.println("Buffered records for device: " + deviceId);
}

// ===== UPLOAD BUFFERED RECORDS =====
bool uploadBuffered() {
  if (!LittleFS.exists(COLLECTOR_PENDING_FILE)) return true;  // nothing to do

  File f = LittleFS.open(COLLECTOR_PENDING_FILE, "r");
  if (!f) return false;

  // Group records by device ID
  struct DeviceRecords {
    String deviceId;
    String compact;
    int    count;
  };

  DeviceRecords groups[10];
  int groupCount = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // First field is device ID
    int comma1 = line.indexOf(',');
    if (comma1 < 0) continue;
    String deviceId = line.substring(0, comma1);
    String record   = line.substring(comma1 + 1);

    // Find or create group
    int groupIdx = -1;
    for (int i = 0; i < groupCount; i++) {
      if (groups[i].deviceId == deviceId) {
        groupIdx = i;
        break;
      }
    }
    if (groupIdx < 0 && groupCount < 10) {
      groups[groupCount].deviceId = deviceId;
      groups[groupCount].compact  = "";
      groups[groupCount].count    = 0;
      groupIdx = groupCount++;
    }
    if (groupIdx < 0) continue;

    if (groups[groupIdx].compact.length() > 0) groups[groupIdx].compact += "|";
    groups[groupIdx].compact += record;
    groups[groupIdx].count++;
  }
  f.close();

  bool allOk = true;

  for (int g = 0; g < groupCount; g++) {
    Serial.println("Uploading " + String(groups[g].count) +
                   " records for device: " + groups[g].deviceId);

    String payload = "device=" + groups[g].deviceId + "&d=" + groups[g].compact;
    payload.replace(" ", "%20");

    WiFiClientSecure secureClient;
    secureClient.setInsecure();

    String url = String(GS_SCRIPT_URL) + "?" + payload;

    HTTPClient http;
    http.begin(secureClient, url);
    http.setTimeout(15000);
    http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

    feedWatchdog();
    int httpCode = http.GET();
    feedWatchdog();

    String response = http.getString();
    http.end();

    Serial.println("HTTP " + String(httpCode) + " — " + response.substring(0, 80));

    if (response.indexOf("\"ok\"") < 0) {
      allOk = false;
      Serial.println("Upload failed for device: " + groups[g].deviceId);
    }
  }

  if (allOk) {
    LittleFS.remove(COLLECTOR_PENDING_FILE);
    Serial.println("All buffered records uploaded");
    setUploadLED(COLOR_GREEN);
  } else {
    Serial.println("Some uploads failed — keeping buffer");
    setUploadLED(COLOR_RED);
  }

  return allOk;
}

// ===== PUBLISH MQTT =====
void publishToMQTT(const String& deviceId, const String& records) {
  if (!connectMQTT()) {
    Serial.println("MQTT connect failed — skipping publish");
    return;
  }

  // Parse last record for latest voltage readings
  float lastTruck = 0, lastBatt = 0;
  int lastPipe = records.lastIndexOf('|');
  String lastRecord = (lastPipe >= 0) ? records.substring(lastPipe + 1) : records;

  int c1 = lastRecord.indexOf(',');
  int c2 = lastRecord.indexOf(',', c1 + 1);
  int c3 = lastRecord.lastIndexOf(',');

  if (c1 > 0 && c2 > 0) {
    lastTruck = lastRecord.substring(c1 + 1, c2).toFloat();
    lastBatt  = lastRecord.substring(c2 + 1, c3).toFloat();
  }

  feedTruckVoltage->publish(lastTruck);
  feedBattVoltage->publish(lastBatt);

  String dbg = "BLE_RX device:" + deviceId +
               " Truck:" + String(lastTruck, 3) +
               " Batt:" + String(lastBatt, 3) +
               " Records:" + String(records.length()) +
               " CollectorV:" + String(COLLECTOR_VERSION);
  feedDebug->publish(dbg.c_str());

  mqtt.disconnect();
  Serial.println("MQTT published for device: " + deviceId);
}

// ===== CHECK MQTT CONTROL MESSAGES =====
void checkControlMessages() {
  if (!connectMQTT()) return;

  // Process incoming messages
  Adafruit_MQTT_Subscribe* sub;
  unsigned long start = millis();

  while (millis() - start < 2000) {
    mqtt.processPackets(500);
    feedWatchdog();

    if (feedControl->lastread[0] != 0) {
      String cmd = String((char*)feedControl->lastread);
      cmd.trim();
      Serial.println("Control message: " + cmd);

      // Clear the message
      memset(feedControl->lastread, 0, sizeof(feedControl->lastread));

      // Parse command — format: command=value:deviceId or command:ALL
      if (cmd.startsWith("debug=on")) {
        String target = cmd.indexOf(':') > 0 ? cmd.substring(cmd.indexOf(':') + 1) : "ALL";
        String bleCmd = "DEBUG_ON";
        if (target == "ALL" || target == FALLBACK_DEVICE_NAME) {
          bleSetPendingCommand(bleCmd);
          Serial.println("Debug ON queued for: " + target);
        }
      } else if (cmd.startsWith("debug=off")) {
        String target = cmd.indexOf(':') > 0 ? cmd.substring(cmd.indexOf(':') + 1) : "ALL";
        if (target == "ALL" || target == FALLBACK_DEVICE_NAME) {
          bleSetPendingCommand("DEBUG_OFF");
          Serial.println("Debug OFF queued for: " + target);
        }
      } else if (cmd.startsWith("sleep=")) {
        int colonIdx = cmd.indexOf(':');
        String value  = (colonIdx > 0) ? cmd.substring(6, colonIdx) : cmd.substring(6);
        String target = (colonIdx > 0) ? cmd.substring(colonIdx + 1) : "ALL";
        if (target == "ALL" || target == FALLBACK_DEVICE_NAME) {
          bleSetPendingCommand("SLEEP:" + value);
          Serial.println("Sleep " + value + "s queued for: " + target);
        }
      } else if (cmd.startsWith("ota")) {
        Serial.println("OTA check requested — will check on next WiFi cycle");
        // OTA handled in main loop
      } else if (cmd == "reboot") {
        Serial.println("Reboot command received");
        delay(500);
        ESP.restart();
      }
    }
    break;
  }

  mqtt.disconnect();
}

// ===== COLLECTOR OTA =====
void checkAndApplyOTA() {
  Serial.println("Checking collector OTA...");

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  http.begin(secureClient, COLLECTOR_OTA_VERSION_URL);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  int httpCode = http.GET();
  feedWatchdog();

  if (httpCode != 200) {
    Serial.println("OTA version check failed — HTTP " + String(httpCode));
    http.end();
    return;
  }

  String remoteVersion = http.getString();
  http.end();
  remoteVersion.trim();

  int localVer  = parseVersion(String(COLLECTOR_VERSION));
  int remoteVer = parseVersion(remoteVersion);

  Serial.println("Collector local: " + String(COLLECTOR_VERSION) +
                 " remote: " + remoteVersion);

  if (remoteVer <= localVer) { Serial.println("Collector firmware up to date"); return; }

  Serial.println("Updating collector to " + remoteVersion + "...");
  setActivityLED(COLOR_PURPLE);

  esp_task_wdt_init(300, false);

  http.begin(secureClient, COLLECTOR_OTA_FIRMWARE_URL);
  http.setTimeout(60000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  httpCode = http.GET();
  feedWatchdog();

  if (httpCode != 200) {
    Serial.println("OTA download failed — HTTP " + String(httpCode));
    http.end();
    esp_task_wdt_init(120, false);
    flashActivity(COLOR_RED);
    return;
  }

  int contentLength = http.getSize();
  if (contentLength <= 0 || !Update.begin(contentLength)) {
    Serial.println("OTA begin failed");
    http.end();
    esp_task_wdt_init(120, false);
    flashActivity(COLOR_RED);
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
      Serial.println("OTA: " + String(written * 100 / contentLength) + "%");
      // Animate progress on activity LEDs
      int ledsOn = (written * LED_ACTIVITY_COUNT) / contentLength;
      for (int i = LED_ACTIVITY_START; i < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; i++) {
        strip.setPixelColor(i, (i - LED_ACTIVITY_START) < ledsOn ? COLOR_PURPLE : COLOR_OFF);
      }
      strip.show();
      lastProgress = millis();
    }
    delay(1);
  }
  http.end();

  if (written != (size_t)contentLength || !Update.end(true)) {
    Serial.println("OTA failed");
    Update.abort();
    esp_task_wdt_init(120, false);
    flashActivity(COLOR_RED);
    return;
  }

  Serial.println("Collector OTA success — rebooting into " + remoteVersion);
  flashActivity(COLOR_GREEN, 5, 100);
  delay(500);
  ESP.restart();
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  esp_task_wdt_init(120, false);
  delay(2000);

  Serial.println("\n=== VoltMon Collector v" + String(COLLECTOR_VERSION) + " ===");

  // Init NeoPixel
  strip.begin();
  strip.setBrightness(VM_NEOPIXEL_BRIGHTNESS);
  strip.clear();
  strip.show();

  // Startup animation — sweep white across all LEDs
  for (int i = 0; i < VM_NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_WHITE);
    strip.show();
    delay(30);
  }
  delay(200);
  strip.clear();
  strip.show();

  // Set initial LED states
  setWiFiLED(COLOR_YELLOW);
  setUploadLED(COLOR_OFF);
  clearActivity();

  // Init LittleFS
  fsReady = initFS();
  if (!fsReady) {
    Serial.println("FATAL: LittleFS failed");
    setActivityLED(COLOR_RED);
    while (1) delay(1000);
  }
  Serial.println("LittleFS OK");

  // Connect WiFi
  if (!connectWiFi()) {
    Serial.println("WARNING: WiFi failed on startup — will retry in loop");
  }

  // Setup MQTT
  setupMQTT();

  // Init BLE collector
  collectorBleInit();

  Serial.println("Collector ready — scanning for VoltMon devices");
  setActivityLED(COLOR_DIM_BLUE);
}

// ===== LOOP =====
void loop() {
  feedWatchdog();

  // Maintain WiFi
  maintainWiFi();

  // Check for BLE data received from VoltMon
  if (bleDataReceived()) {
    String deviceId = bleGetDeviceId();
    String records  = bleGetRecords();
    bool   isEmpty  = bleGotEmpty();

    if (isEmpty) {
      Serial.println("BLE: VoltMon has no pending records");
      flashActivity(COLOR_CYAN, 2, 100);
    } else {
      Serial.println("BLE: Received " + String(records.length()) +
                     " bytes from " + deviceId);
      setActivityLED(COLOR_WHITE);

      // Buffer records first — collector owns them now
      bufferRecords(deviceId, records);

      // Attempt immediate upload
      if (wifiConnected) {
        setActivityLED(COLOR_CYAN);
        bool ok = uploadBuffered();
        if (ok) {
          flashActivity(COLOR_GREEN, 3, 150);
          publishToMQTT(deviceId, records);
        } else {
          flashActivity(COLOR_RED, 3, 150);
          setUploadLED(COLOR_YELLOW);  // pending
        }
      } else {
        Serial.println("WiFi down — records buffered for later upload");
        setUploadLED(COLOR_YELLOW);
        flashActivity(COLOR_RED, 2, 200);
      }
    }

    bleClearReceived();
    setActivityLED(COLOR_DIM_BLUE);  // back to idle/scanning
  }

  // Retry buffered uploads if WiFi is up
  static unsigned long lastUploadRetry = 0;
  if (wifiConnected && LittleFS.exists(COLLECTOR_PENDING_FILE)) {
    if (millis() - lastUploadRetry > COLLECTOR_UPLOAD_RETRY_MS) {
      Serial.println("Retrying buffered upload...");
      setActivityLED(COLOR_CYAN);
      uploadBuffered();
      setActivityLED(COLOR_DIM_BLUE);
      lastUploadRetry = millis();
    }
  }

  // Check MQTT control messages every 30 seconds
  static unsigned long lastMQTTCheck = 0;
  if (wifiConnected && millis() - lastMQTTCheck > 30000) {
    checkControlMessages();
    lastMQTTCheck = millis();
  }

  // Check collector OTA once per hour
  static unsigned long lastOTACheck = 0;
  if (wifiConnected && millis() - lastOTACheck > 3600000) {
    setActivityLED(COLOR_PURPLE);
    checkAndApplyOTA();
    setActivityLED(COLOR_DIM_BLUE);
    lastOTACheck = millis();
  }

  // Idle animation — slow pulse on activity LEDs when scanning
  static unsigned long lastPulse = 0;
  if (!collectorConnected && millis() - lastPulse > 3000) {
    animatePulse(COLOR_BLUE, 15, 20);
    lastPulse = millis();
  }

  delay(100);
}