#include "Arduino.h"
#include <WiFi.h>
#include "esp_partition.h"
#include "esp_ota_ops.h"
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
  for (int i = LED_WIFI_START; i < LED_WIFI_START + LED_WIFI_COUNT; i++)
    strip.setPixelColor(i, color);
  strip.show();
}

void setUploadLED(uint32_t color) {
  for (int i = LED_UPLOAD_START; i < LED_UPLOAD_START + LED_UPLOAD_COUNT; i++)
    strip.setPixelColor(i, color);
  strip.show();
}

void setActivityLED(uint32_t color) {
  for (int i = LED_ACTIVITY_START; i < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; i++)
    strip.setPixelColor(i, color);
  strip.show();
}

void clearActivity() { setActivityLED(COLOR_OFF); }

void animatePulse(uint32_t color, int steps = 20, int delayMs = 30) {
  for (int i = 0; i <= steps; i++) {
    uint8_t r = ((color >> 16) & 0xFF) * i / steps;
    uint8_t g = ((color >> 8)  & 0xFF) * i / steps;
    uint8_t b = ((color)       & 0xFF) * i / steps;
    setActivityLED(strip.Color(r, g, b));
    delay(delayMs);
  }
  for (int i = steps; i >= 0; i--) {
    uint8_t r = ((color >> 16) & 0xFF) * i / steps;
    uint8_t g = ((color >> 8)  & 0xFF) * i / steps;
    uint8_t b = ((color)       & 0xFF) * i / steps;
    setActivityLED(strip.Color(r, g, b));
    delay(delayMs);
  }
}

void flashActivity(uint32_t color, int times = 3, int delayMs = 150) {
  for (int i = 0; i < times; i++) {
    setActivityLED(color);
    delay(delayMs);
    setActivityLED(COLOR_OFF);
    delay(delayMs);
  }
}

// ===== GLOBALS =====
bool fsReady          = false;
bool mqttReady        = false;
bool wifiConnected    = false;
bool hasPendingBuffer = false;

// ===== VOLTMON OTA CACHE — declared extern in ble_client.h =====
String cachedVoltMonVersion  = "";
size_t cachedFirmwareSize    = 0;
bool   voltmonFirmwareCached = false;
String lastKnownVoltMonVer   = "";  // last version VoltMon reported via BLE
#define VOLTMON_CACHE_FILE "/voltmon_cached.txt"

// Parse version string into integer for comparison: "2.1.9" -> 20109
int parseVersionInt(const String& v) {
  int maj = 0, min = 0, pat = 0;
  sscanf(v.c_str(), "%d.%d.%d", &maj, &min, &pat);
  return maj * 10000 + min * 100 + pat;
}

// Returns true if vA is strictly newer than vB
bool versionNewer(const String& vA, const String& vB) {
  return parseVersionInt(vA) > parseVersionInt(vB);
}

void saveVoltMonCache() {
  File f = LittleFS.open(VOLTMON_CACHE_FILE, "w", true);
  if (!f) return;
  f.println(cachedVoltMonVersion);
  f.println(cachedFirmwareSize);
  f.println(voltmonFirmwareCached ? "1" : "0");
  f.println(lastKnownVoltMonVer);
  f.close();
}

void loadVoltMonCache() {
  if (!LittleFS.exists(VOLTMON_CACHE_FILE)) return;
  File f = LittleFS.open(VOLTMON_CACHE_FILE, "r");
  if (!f) return;
  cachedVoltMonVersion = f.readStringUntil('\n'); cachedVoltMonVersion.trim();
  cachedFirmwareSize   = f.readStringUntil('\n').toInt();
  String cached        = f.readStringUntil('\n'); cached.trim();
  voltmonFirmwareCached = (cached == "1");
  lastKnownVoltMonVer  = f.readStringUntil('\n'); lastKnownVoltMonVer.trim();
  f.close();
  Serial.println("VoltMon cache: firmware=v" + cachedVoltMonVersion +
                 " lastSeen=v" + lastKnownVoltMonVer +
                 " cached=" + String(voltmonFirmwareCached ? "yes" : "no"));
}

#define COLLECTOR_PENDING_FILE  "/col_pending.csv"

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
    f = LittleFS.open("/fstest", "w");
    if (!f) return false;
  }
  f.close();
  LittleFS.remove("/fstest");

  File pf = LittleFS.open(COLLECTOR_PENDING_FILE, "r");
  if (pf && pf.size() > 0) {
    hasPendingBuffer = true;
    Serial.println("Found existing pending buffer — " + String(pf.size()) + " bytes");
  }
  if (pf) pf.close();

  return true;
}

// ===== MQTT SETUP =====
void setupMQTT() {
  mqtt.~Adafruit_MQTT_Client();
  new (&mqtt) Adafruit_MQTT_Client(&mqttWifiClient, AIO_SERVER, AIO_PORT,
                                    AIO_USERNAME, AIO_KEY);

  snprintf(TOPIC_TRUCK,   sizeof(TOPIC_TRUCK),   "%s/feeds/truck-voltage",   AIO_USERNAME);
  snprintf(TOPIC_BATT,    sizeof(TOPIC_BATT),    "%s/feeds/device-voltage",  AIO_USERNAME);
  snprintf(TOPIC_DEBUG,   sizeof(TOPIC_DEBUG),   "%s/feeds/debug-messages",  AIO_USERNAME);
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
  memset(feedControl->lastread, 0, sizeof(feedControl->lastread));
  mqttReady = true;
}

// ===== WIFI =====
bool connectWiFi() {
  Serial.println("WiFi connecting...");
  setWiFiLED(COLOR_YELLOW);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSIDS[0], WIFI_PASSWORDS[0]);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    feedWatchdog();
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString() +
                   " RSSI:" + String(WiFi.RSSI()));
    setWiFiLED(COLOR_GREEN);
    wifiConnected = true;
    return true;
  }

  Serial.println("WiFi failed — status: " + String(WiFi.status()));
  setWiFiLED(COLOR_RED);
  wifiConnected = false;
  return false;
}

void checkVoltMonFirmwareVersion();  // forward declaration
bool downloadVoltMonFirmware();       // forward declaration

void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
    setWiFiLED(COLOR_YELLOW);
    Serial.println("WiFi lost — reconnecting...");
    WiFi.disconnect();
    delay(1000);
    if (connectWiFi() && cachedVoltMonVersion.length() == 0) {
      checkVoltMonFirmwareVersion();
      bleSetOtaAvailable();
      bleUpdateAdvertising();
    }
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

// ===== VOLTMON FIRMWARE VERSION CHECK =====
// Fetches version.txt and Content-Length of firmware binary
// Updates cachedVoltMonVersion and cachedFirmwareSize globals
void checkVoltMonFirmwareVersion() {
  if (!wifiConnected) return;

  Serial.println("Checking VoltMon firmware version...");

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
    Serial.println("VoltMon version check failed — HTTP " + String(httpCode));
    http.end();
    return;
  }

  String remoteVersion = http.getString();
  http.end();
  remoteVersion.trim();

  // Only download if GitHub version is newer than what VoltMon last reported
  if (lastKnownVoltMonVer.length() > 0 &&
      !versionNewer(remoteVersion, lastKnownVoltMonVer)) {
    Serial.println("VoltMon on v" + lastKnownVoltMonVer +
                   " — GitHub v" + remoteVersion + " not newer, skipping");
    return;
  }

  // Already have this version downloaded
  if (remoteVersion == cachedVoltMonVersion && cachedFirmwareSize > 0 && voltmonFirmwareCached) {
    Serial.println("VoltMon firmware already downloaded: v" + remoteVersion);
    bleSetOtaAvailable();
    return;
  }

  // Fetch firmware to get size (Content-Length from headers)
  http.begin(secureClient, VOLTMON_OTA_FIRMWARE_URL);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  httpCode = http.GET();
  feedWatchdog();

  if (httpCode == 200) {
    int size = http.getSize();
    http.end();

    if (size > 0) {
      bool versionChanged = (cachedVoltMonVersion != remoteVersion);
      cachedVoltMonVersion = remoteVersion;
      cachedFirmwareSize   = (size_t)size;
      if (versionChanged) voltmonFirmwareCached = false;
      Serial.println("VoltMon firmware cached: v" + cachedVoltMonVersion +
                     " size=" + String(cachedFirmwareSize) + " bytes");
      if (versionChanged || !voltmonFirmwareCached) {
        Serial.println("VoltMon firmware changed — downloading to partition...");
        downloadVoltMonFirmware();
      }
      bleSetOtaAvailable();
    } else {
      Serial.println("VoltMon firmware size unknown");
      http.end();
    }
  } else {
    Serial.println("VoltMon firmware fetch failed — HTTP " + String(httpCode));
    http.end();
  }
}

// ===== STREAM OTA FIRMWARE TO VOLTMON OVER BLE =====
// ===== DOWNLOAD VOLTMON FIRMWARE TO INACTIVE OTA PARTITION =====
bool downloadVoltMonFirmware() {
  if (cachedFirmwareSize == 0 || cachedVoltMonVersion.length() == 0) {
    Serial.println("OTA download: no firmware version cached");
    return false;
  }

  // Get the inactive OTA partition — safe to write to
  const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
  if (!partition) {
    Serial.println("OTA download: no inactive partition found");
    return false;
  }

  Serial.println("OTA download: target partition: " + String(partition->label) +
                 " size=" + String(partition->size));

  if (cachedFirmwareSize > partition->size) {
    Serial.println("OTA download: firmware too large for partition");
    return false;
  }

  // Ensure WiFi before long download
  maintainWiFi();
  if (!wifiConnected) {
    Serial.println("OTA download: WiFi not available");
    return false;
  }

  Serial.println("OTA download: fetching v" + cachedVoltMonVersion + "...");
  setActivityLED(COLOR_PURPLE);

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  String versionedUrl = String(VOLTMON_OTA_FIRMWARE_URL) + "?t=" + String(millis());
  http.begin(secureClient, versionedUrl);
  http.setTimeout(60000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  feedWatchdog();
  int httpCode = http.GET();
  feedWatchdog();

  if (httpCode != 200) {
    Serial.println("OTA download: HTTP failed — " + String(httpCode));
    http.end();
    flashActivity(COLOR_RED);
    return false;
  }

  int contentLength = http.getSize();
  Serial.println("OTA download: content-length=" + String(contentLength));

  if (contentLength <= 0 || (size_t)contentLength != cachedFirmwareSize) {
    Serial.println("OTA download: size mismatch — expected " +
                   String(cachedFirmwareSize) + " got " + String(contentLength));
    http.end();
    return false;
  }

  // Erase partition
  Serial.println("OTA download: erasing partition...");
  esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);
  if (err != ESP_OK) {
    Serial.println("OTA download: erase failed — " + String(esp_err_to_name(err)));
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();

  const size_t BUF_SIZE = 4096;
  uint8_t      buf[BUF_SIZE];
  size_t       written       = 0;
  uint32_t     offset        = 0;
  unsigned long lastWdog     = millis();
  unsigned long lastProgress = millis();

  while (http.connected() && written < (size_t)contentLength) {
    size_t available = stream->available();
    if (available == 0) { delay(5); continue; }

    size_t toRead    = min(available, BUF_SIZE);
    size_t bytesRead = stream->readBytes(buf, toRead);
    if (bytesRead == 0) continue;

    err = esp_partition_write(partition, offset, buf, bytesRead);
    if (err != ESP_OK) {
      Serial.println("OTA download: write failed at " + String(offset));
      http.end();
      voltmonFirmwareCached = false;
      return false;
    }

    offset  += bytesRead;
    written += bytesRead;

    if (millis() - lastWdog > 5000)    { feedWatchdog(); lastWdog = millis(); }
    if (millis() - lastProgress > 5000) {
      int pct = (contentLength > 0) ? (written * 100 / contentLength) : 0;
      Serial.println("OTA download: " + String(pct) + "% (" +
                     String(written) + "/" + String(contentLength) + ")");
      int ledsOn = (written * LED_ACTIVITY_COUNT) / contentLength;
      for (int i = LED_ACTIVITY_START; i < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; i++) {
        strip.setPixelColor(i, (i - LED_ACTIVITY_START) < ledsOn ? COLOR_PURPLE : COLOR_OFF);
      }
      strip.show();
      lastProgress = millis();
    }
  }

  http.end();

  if (written != (size_t)contentLength) {
    Serial.println("OTA download: incomplete — " + String(written) +
                   "/" + String(contentLength));
    voltmonFirmwareCached = false;
    return false;
  }

  voltmonFirmwareCached = true;
  saveVoltMonCache();
  Serial.println("OTA download: complete — " + String(written) + " bytes written to " +
                 String(partition->label));
  flashActivity(COLOR_GREEN, 3, 100);
  setActivityLED(COLOR_DIM_BLUE);
  return true;
}

// ===== PUSH OTA FIRMWARE TO VOLTMON =====
// Collector scans for "VoltMon-OTA", connects as client, writes chunks via WRITE_NR
// VoltMon is the server — onWrite callback handles each chunk — no drops possible
bool pushOTAToVoltMon() {
  if (!voltmonFirmwareCached || cachedFirmwareSize == 0) {
    Serial.println("OTA push: no firmware cached");
    return false;
  }

  const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
  if (!partition) {
    Serial.println("OTA push: partition not found");
    return false;
  }

  Serial.println("OTA push: scanning for VoltMon-OTA...");
  setActivityLED(COLOR_PURPLE);

  // Stop advertising while we scan — can't advertise and scan simultaneously
  NimBLEDevice::getAdvertising()->stop();

  // Give VoltMon time to start advertising
  delay(2000);

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
  NimBLEScanResults results = scan->start(8, false);

  NimBLEAdvertisedDevice* voltmon = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    if (String(device.getName().c_str()) == "VoltMon-OTA") {
      voltmon = new NimBLEAdvertisedDevice(device);
      Serial.println("OTA push: VoltMon-OTA found");
      break;
    }
  }
  scan->clearResults();

  if (!voltmon) {
    Serial.println("OTA push: VoltMon-OTA not found");
    flashActivity(COLOR_RED, 3, 100);
    return false;
  }

  NimBLEClient* client = NimBLEDevice::createClient();
  bool connected = client->connect(voltmon);
  delete voltmon; voltmon = nullptr;

  if (!connected) {
    Serial.println("OTA push: Connection failed");
    NimBLEDevice::deleteClient(client);
    return false;
  }
  Serial.println("OTA push: Connected to VoltMon");

  NimBLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) {
    client->disconnect(); NimBLEDevice::deleteClient(client); return false;
  }

  NimBLERemoteCharacteristic* otaChar = service->getCharacteristic(BLE_OTA_CHAR_UUID);
  if (!otaChar) {
    Serial.println("OTA push: OTA characteristic not found");
    client->disconnect(); NimBLEDevice::deleteClient(client); return false;
  }

  // Extended connection params for long transfer
  client->setConnectionParams(6, 6, 0, 600);

  Serial.println("OTA push: Pushing " + String(cachedFirmwareSize) + " bytes...");

  const size_t CHUNK_SIZE = 480;
  uint8_t      buf[CHUNK_SIZE];
  size_t       totalSent     = 0;
  uint32_t     offset        = 0;
  unsigned long lastWdog     = millis();
  unsigned long lastProgress = millis();

  while (totalSent < cachedFirmwareSize && client->isConnected()) {
    size_t remaining = cachedFirmwareSize - totalSent;
    size_t toRead    = min(remaining, CHUNK_SIZE);

    esp_err_t err = esp_partition_read(partition, offset, buf, toRead);
    if (err != ESP_OK) {
      Serial.println("OTA push: read failed at " + String(offset));
      break;
    }

    // WRITE_NR — no response needed, VoltMon onWrite fires for every packet
    if (!otaChar->writeValue(buf, toRead, false)) {
      // Brief backoff if write fails — BLE queue may be full
      delay(10);
      if (!otaChar->writeValue(buf, toRead, false)) {
        Serial.println("OTA push: write failed at " + String(totalSent));
        break;
      }
    }

    totalSent += toRead;
    offset    += toRead;

    // Small delay to let VoltMon process and write to flash
    delay(5);

    if (millis() - lastWdog > 5000) { feedWatchdog(); lastWdog = millis(); }

    if (millis() - lastProgress > 10000) {
      int pct = totalSent * 100 / cachedFirmwareSize;
      Serial.println("OTA push: " + String(pct) + "% (" +
                     String(totalSent) + "/" + String(cachedFirmwareSize) + ")");
      int ledsOn = (totalSent * LED_ACTIVITY_COUNT) / cachedFirmwareSize;
      for (int i = LED_ACTIVITY_START; i < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; i++) {
        strip.setPixelColor(i, (i - LED_ACTIVITY_START) < ledsOn ? COLOR_PURPLE : COLOR_OFF);
      }
      strip.show();
      lastProgress = millis();
    }
  }

  bool success = (totalSent >= cachedFirmwareSize);

  if (success) {
    // Send END marker
    otaChar->writeValue("END", false);
    delay(100);
    Serial.println("OTA push: Complete — " + String(totalSent) + " bytes sent");
    flashActivity(COLOR_GREEN, 5, 100);
    // Clear cache
    lastKnownVoltMonVer   = cachedVoltMonVersion;  // VoltMon now has this version
    voltmonFirmwareCached = false;
    cachedVoltMonVersion  = "";
    cachedFirmwareSize    = 0;
    saveVoltMonCache();
    bleSetOtaAvailable();
    if (connectMQTT()) {
      feedDebug->publish(("OTA_PUSH_OK_to_" + bleOtaTargetDevice()).c_str());
      mqtt.disconnect();
    }
  } else {
    Serial.println("OTA push: Incomplete — " + String(totalSent) +
                   "/" + String(cachedFirmwareSize));
    flashActivity(COLOR_RED, 5, 100);
  }

  client->disconnect();
  NimBLEDevice::deleteClient(client);

  // Restart collector advertising
  NimBLEDevice::getAdvertising()->start();
  Serial.println("OTA push: collector advertising restarted");

  setActivityLED(COLOR_DIM_BLUE);
  return success;
}

void streamOTAToVoltMon() {
  if (!voltmonFirmwareCached || cachedFirmwareSize == 0) {
    Serial.println("OTA stream: no firmware cached — aborting");
    return;
  }

  const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
  if (!partition) {
    Serial.println("OTA stream: partition not found");
    return;
  }

  if (!collectorConnected) {
    Serial.println("OTA stream: VoltMon not connected");
    return;
  }

  Serial.println("OTA stream: pull mode — " + String(cachedFirmwareSize) +
                 " bytes from " + String(partition->label));
  setActivityLED(COLOR_PURPLE);

  const size_t CHUNK_SIZE = 480;
  uint8_t      buf[CHUNK_SIZE];
  size_t       totalSent   = 0;
  uint32_t     offset      = 0;
  unsigned long lastWdog   = millis();
  unsigned long lastProgress = millis();

  bleOtaClearNext();

  while (totalSent < cachedFirmwareSize && collectorConnected) {

    // Wait for VoltMon to request next chunk — it just wrote the previous one to flash
    unsigned long nextWait = millis();
    while (!bleOtaNextRequested() && collectorConnected && millis() - nextWait < 10000) {
      feedWatchdog();
      delay(1);
    }

    if (!bleOtaNextRequested()) {
      Serial.println("OTA stream: NEXT timeout at " + String(totalSent));
      break;
    }
    bleOtaClearNext();

    // Read chunk from local partition — instant, no network
    size_t remaining = cachedFirmwareSize - totalSent;
    size_t toRead    = min(remaining, CHUNK_SIZE);

    esp_err_t err = esp_partition_read(partition, offset, buf, toRead);
    if (err != ESP_OK) {
      Serial.println("OTA stream: partition read failed at " + String(offset));
      break;
    }

    // Send chunk via indication
    collectorOtaChar->setValue(buf, toRead);
    collectorOtaChar->indicate();
    totalSent += toRead;
    offset    += toRead;

    if (millis() - lastWdog > 5000) { feedWatchdog(); lastWdog = millis(); }

    if (millis() - lastProgress > 10000) {
      int pct = totalSent * 100 / cachedFirmwareSize;
      Serial.println("OTA stream: " + String(pct) + "% (" +
                     String(totalSent) + "/" + String(cachedFirmwareSize) + ")");
      int ledsOn = (totalSent * LED_ACTIVITY_COUNT) / cachedFirmwareSize;
      for (int i = LED_ACTIVITY_START; i < LED_ACTIVITY_START + LED_ACTIVITY_COUNT; i++) {
        strip.setPixelColor(i, (i - LED_ACTIVITY_START) < ledsOn ? COLOR_PURPLE : COLOR_OFF);
      }
      strip.show();
      lastProgress = millis();
    }
  }

  if (totalSent >= cachedFirmwareSize) {
    Serial.println("OTA stream: Complete — " + String(totalSent) + " bytes sent");
    flashActivity(COLOR_GREEN, 5, 100);
    setUploadLED(COLOR_GREEN);
    lastKnownVoltMonVer   = cachedVoltMonVersion;  // VoltMon now has this version
    voltmonFirmwareCached = false;
    cachedVoltMonVersion  = "";
    cachedFirmwareSize    = 0;
    saveVoltMonCache();
    bleSetOtaAvailable();
    if (connectMQTT()) {
      feedDebug->publish(("OTA_PROXY_OK_to_" + bleOtaTargetDevice()).c_str());
      mqtt.disconnect();
    }
  } else {
    Serial.println("OTA stream: Incomplete — " + String(totalSent) +
                   "/" + String(cachedFirmwareSize));
    flashActivity(COLOR_RED, 5, 100);
  }
}

// ===== BUFFER RECORDS =====
void bufferRecords(const String& deviceId, const String& records) {
  if (deviceId.length() == 0 || records.length() == 0) return;

  if (!LittleFS.exists(COLLECTOR_PENDING_FILE)) {
    File init = LittleFS.open(COLLECTOR_PENDING_FILE, "w");
    if (init) init.close();
  }

  File f = LittleFS.open(COLLECTOR_PENDING_FILE, "a");
  if (!f) {
    Serial.println("ERROR: Cannot open collector pending for write");
    return;
  }

  int start   = 0;
  int written = 0;
  while (start < (int)records.length()) {
    int end    = records.indexOf('|', start);
    if (end < 0) end = records.length();
    String record = records.substring(start, end);
    record.trim();
    if (record.length() > 0) {
      f.println(deviceId + "," + record);
      written++;
    }
    start = end + 1;
  }
  f.close();

  hasPendingBuffer = true;
  Serial.println("Buffered " + String(written) + " records for: " + deviceId);
}

// ===== UPLOAD BUFFERED RECORDS =====
bool uploadBuffered() {
  if (!hasPendingBuffer) return true;

  File f = LittleFS.open(COLLECTOR_PENDING_FILE, "r");
  if (!f) { hasPendingBuffer = false; return true; }

  if (f.size() == 0) {
    f.close();
    LittleFS.remove(COLLECTOR_PENDING_FILE);
    hasPendingBuffer = false;
    return true;
  }

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

    int comma1 = line.indexOf(',');
    if (comma1 < 0) continue;
    String deviceId = line.substring(0, comma1);
    String record   = line.substring(comma1 + 1);

    int groupIdx = -1;
    for (int i = 0; i < groupCount; i++) {
      if (groups[i].deviceId == deviceId) { groupIdx = i; break; }
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

  if (groupCount == 0) {
    LittleFS.remove(COLLECTOR_PENDING_FILE);
    hasPendingBuffer = false;
    return true;
  }

  bool allOk = true;

  for (int g = 0; g < groupCount; g++) {
    Serial.println("Uploading " + String(groups[g].count) +
                   " records for: " + groups[g].deviceId);

    String payload = "device=" + groups[g].deviceId + "&d=" + groups[g].compact;
    payload.replace(" ", "%20");

    WiFiClientSecure secureClient;
    secureClient.setInsecure();

    HTTPClient http;
    http.begin(secureClient, String(GS_SCRIPT_URL) + "?" + payload);
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
    }
  }

  if (allOk) {
    LittleFS.remove(COLLECTOR_PENDING_FILE);
    hasPendingBuffer = false;
    Serial.println("All records uploaded and buffer cleared");
    setUploadLED(COLOR_GREEN);
  } else {
    Serial.println("Some uploads failed — keeping buffer");
    setUploadLED(COLOR_RED);
  }

  return allOk;
}

// ===== PUBLISH MQTT =====
void publishToMQTT(const String& deviceId, const String& records) {
  if (!connectMQTT()) return;

  float lastTruck = 0, lastBatt = 0;
  int lastPipe   = records.lastIndexOf('|');
  String lastRec = (lastPipe >= 0) ? records.substring(lastPipe + 1) : records;

  int c1 = lastRec.indexOf(',');
  int c2 = lastRec.indexOf(',', c1 + 1);
  int c3 = lastRec.lastIndexOf(',');

  if (c1 > 0 && c2 > 0) {
    lastTruck = lastRec.substring(c1 + 1, c2).toFloat();
    lastBatt  = lastRec.substring(c2 + 1, c3).toFloat();
  }

  feedTruckVoltage->publish(lastTruck);
  feedBattVoltage->publish(lastBatt);

  String dbg = "BLE_RX device:" + deviceId +
               " Truck:" + String(lastTruck, 3) +
               " Batt:" + String(lastBatt, 3) +
               " v" + String(COLLECTOR_VERSION);
  feedDebug->publish(dbg.c_str());

  mqtt.disconnect();
}

// ===== CHECK MQTT CONTROL MESSAGES =====
void checkControlMessages() {
  if (!connectMQTT()) return;

  unsigned long start = millis();
  while (millis() - start < 2000) {
    mqtt.processPackets(500);
    feedWatchdog();

    if (feedControl->lastread[0] != 0) {
      String cmd = String((char*)feedControl->lastread);
      cmd.trim();
      Serial.println("Control message: " + cmd);
      memset(feedControl->lastread, 0, sizeof(feedControl->lastread));

      if (cmd.startsWith("debug=on")) {
        String target = cmd.indexOf(':') > 0 ? cmd.substring(cmd.indexOf(':') + 1) : "ALL";
        bleSetPendingCommand("DEBUG_ON");
      } else if (cmd.startsWith("debug=off")) {
        bleSetPendingCommand("DEBUG_OFF");
      } else if (cmd.startsWith("sleep=")) {
        int colonIdx  = cmd.indexOf(':');
        String value  = (colonIdx > 0) ? cmd.substring(6, colonIdx) : cmd.substring(6);
        bleSetPendingCommand("SLEEP:" + value);
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

  if (remoteVer <= localVer) { Serial.println("Collector v" + String(COLLECTOR_VERSION) + " up to date"); return; }

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
    return;
  }

  WiFiClient* stream       = http.getStreamPtr();
  size_t written           = 0;
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

  strip.begin();
  strip.setBrightness(VM_NEOPIXEL_BRIGHTNESS);
  strip.clear();
  strip.show();

  for (int i = 0; i < VM_NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, COLOR_WHITE);
    strip.show();
    delay(30);
  }
  delay(200);
  strip.clear();
  strip.show();

  setWiFiLED(COLOR_YELLOW);
  setUploadLED(COLOR_OFF);
  clearActivity();

  fsReady = initFS();
  if (!fsReady) {
    Serial.println("FATAL: LittleFS failed");
    setActivityLED(COLOR_RED);
    while (1) delay(1000);
  }
  Serial.println("LittleFS OK");
  loadVoltMonCache();

  if (!connectWiFi()) {
    Serial.println("WARNING: WiFi failed on startup — will retry in loop");
  }

  setupMQTT();

  // Check VoltMon firmware version on startup
  if (wifiConnected) {
    checkVoltMonFirmwareVersion();
  }

  collectorBleInit();
  // Download firmware to partition if available, then advertise
  if (cachedVoltMonVersion.length() > 0 && !voltmonFirmwareCached) {
    downloadVoltMonFirmware();
  }
  bleSetOtaAvailable();
  bleUpdateAdvertising();

  Serial.println("Collector ready — waiting for VoltMon");
  if (cachedVoltMonVersion.length() > 0) {
    Serial.println("VoltMon firmware cached: v" + cachedVoltMonVersion +
                   " (" + String(cachedFirmwareSize) + " bytes)");
  }
  setActivityLED(COLOR_DIM_BLUE);
}

// ===== LOOP =====
void loop() {
  feedWatchdog();

  maintainWiFi();

  // Handle BLE data received from VoltMon
  if (bleDataReceived()) {
    String deviceId = bleGetDeviceId();
    String records  = bleGetRecords();
    bool   isEmpty  = bleGotEmpty();

    Serial.println("BLE data — deviceId:[" + deviceId +
                   "] len:" + String(records.length()) +
                   " empty:" + String(isEmpty));

    bleClearReceived();



    // ===== THEN HANDLE RECORDS =====
    if (isEmpty) {
      Serial.println("BLE: VoltMon has no pending records");
      flashActivity(COLOR_CYAN, 2, 100);
    } else {
      setActivityLED(COLOR_WHITE);
      bufferRecords(deviceId, records);

      if (wifiConnected) {
        setActivityLED(COLOR_CYAN);
        bool ok = uploadBuffered();
        if (ok) {
          flashActivity(COLOR_GREEN, 3, 150);
          publishToMQTT(deviceId, records);
        } else {
          flashActivity(COLOR_RED, 3, 150);
          setUploadLED(COLOR_YELLOW);
        }
      } else {
        Serial.println("WiFi down — records buffered");
        setUploadLED(COLOR_YELLOW);
        flashActivity(COLOR_RED, 2, 200);
      }
    }

    setActivityLED(COLOR_DIM_BLUE);
  }

  // OTA push — VoltMon has switched to server mode advertising "VoltMon-OTA"
  // Collector scans, connects as client, pushes chunks via WRITE_NR
  if (bleOtaReadyReceived()) {
    bleOtaClearReady();
    bleOtaClearPending();
    Serial.println("OTA push: VoltMon in server mode — pushing firmware");
    pushOTAToVoltMon();
    setActivityLED(COLOR_DIM_BLUE);
  }

  // Retry buffered uploads
  static unsigned long lastUploadRetry = 0;
  if (wifiConnected && hasPendingBuffer) {
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

  // Check VoltMon firmware version every hour
  static unsigned long lastVoltMonVersionCheck = 0;
  if (wifiConnected && millis() - lastVoltMonVersionCheck > 3600000UL) {
    String prevVersion = cachedVoltMonVersion;
    checkVoltMonFirmwareVersion();
    if (cachedVoltMonVersion != prevVersion) {
      bleUpdateAdvertising();
    }
    lastVoltMonVersionCheck = millis();
  }

  // Check collector OTA once per hour (offset by 5 minutes)
  static unsigned long lastOTACheck = 0;
  if (wifiConnected && millis() - lastOTACheck > 3600000UL) {
    setActivityLED(COLOR_PURPLE);
    checkAndApplyOTA();
    setActivityLED(COLOR_DIM_BLUE);
    lastOTACheck = millis() + 300000UL; // offset from VoltMon version check
  }

  // Idle pulse animation
  static unsigned long lastPulse = 0;
  if (!collectorConnected && millis() - lastPulse > 3000) {
    animatePulse(COLOR_BLUE, 15, 20);
    lastPulse = millis();
  }

  delay(100);
}