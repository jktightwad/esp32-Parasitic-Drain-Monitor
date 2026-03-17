#pragma once

#include <LittleFS.h>
#include <ArduinoJson.h>

// ===== FILE PATHS =====
#define CONFIG_FILE       "/config.json"
#define WIFI_FILE         "/wifi.json"
#define PENDING_FILE      "/pending.csv"
#define ARCHIVE_FILE      "/archive.csv"
#define STATE_FILE        "/state.json"
#define DEBUG_FILE        "/debug.log"

#define MAX_WIFI_NETWORKS  10
#define MAX_DEVICE_NAME    32
#define MAX_SSID_LEN       33
#define MAX_PASS_LEN       65
#define MAX_AIO_LEN        64

// ===== DEVICE CONFIG =====
struct DeviceConfig {
  char deviceName[MAX_DEVICE_NAME] = "";
  char aioUsername[MAX_AIO_LEN]    = "";
  char aioKey[MAX_AIO_LEN]         = "";
  bool configured                  = false;  // true after first setup complete
};

// ===== WIFI CREDENTIAL ENTRY =====
struct WiFiCredential {
  char ssid[MAX_SSID_LEN] = "";
  char pass[MAX_PASS_LEN] = "";
};

// ===== WIFI CREDENTIAL LIST =====
struct WiFiCredentials {
  WiFiCredential networks[MAX_WIFI_NETWORKS];
  int count = 0;
};

// ===== LOAD DEVICE CONFIG =====
bool loadConfig(DeviceConfig& cfg) {
  if (!LittleFS.exists(CONFIG_FILE)) return false;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) return false;

  strlcpy(cfg.deviceName, doc["deviceName"] | "", sizeof(cfg.deviceName));
  strlcpy(cfg.aioUsername, doc["aioUsername"] | "", sizeof(cfg.aioUsername));
  strlcpy(cfg.aioKey,      doc["aioKey"] | "",      sizeof(cfg.aioKey));
  cfg.configured = doc["configured"] | false;

  return true;
}

// ===== SAVE DEVICE CONFIG =====
bool saveConfig(const DeviceConfig& cfg) {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return false;

  JsonDocument doc;
  doc["deviceName"]  = cfg.deviceName;
  doc["aioUsername"] = cfg.aioUsername;
  doc["aioKey"]      = cfg.aioKey;
  doc["configured"]  = cfg.configured;

  serializeJson(doc, f);
  f.close();
  return true;
}

// ===== LOAD WIFI CREDENTIALS =====
bool loadWiFiCredentials(WiFiCredentials& creds) {
  creds.count = 0;
  if (!LittleFS.exists(WIFI_FILE)) return false;

  File f = LittleFS.open(WIFI_FILE, "r");
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) return false;

  JsonArray arr = doc["networks"].as<JsonArray>();
  for (JsonObject net : arr) {
    if (creds.count >= MAX_WIFI_NETWORKS) break;
    strlcpy(creds.networks[creds.count].ssid, net["ssid"] | "", MAX_SSID_LEN);
    strlcpy(creds.networks[creds.count].pass, net["pass"] | "", MAX_PASS_LEN);
    creds.count++;
  }

  return creds.count > 0;
}

// ===== SAVE WIFI CREDENTIALS =====
bool saveWiFiCredentials(const WiFiCredentials& creds) {
  File f = LittleFS.open(WIFI_FILE, "w");
  if (!f) return false;

  JsonDocument doc;
  JsonArray arr = doc["networks"].to<JsonArray>();

  for (int i = 0; i < creds.count; i++) {
    JsonObject net = arr.add<JsonObject>();
    net["ssid"] = creds.networks[i].ssid;
    net["pass"] = creds.networks[i].pass;
  }

  serializeJson(doc, f);
  f.close();
  return true;
}

// ===== ADD WIFI NETWORK =====
// Returns false if already at max capacity or SSID already exists
bool addWiFiNetwork(WiFiCredentials& creds, const char* ssid, const char* pass) {
  if (creds.count >= MAX_WIFI_NETWORKS) return false;

  // Update password if SSID already exists
  for (int i = 0; i < creds.count; i++) {
    if (strcmp(creds.networks[i].ssid, ssid) == 0) {
      strlcpy(creds.networks[i].pass, pass, MAX_PASS_LEN);
      return saveWiFiCredentials(creds);
    }
  }

  strlcpy(creds.networks[creds.count].ssid, ssid, MAX_SSID_LEN);
  strlcpy(creds.networks[creds.count].pass, pass, MAX_PASS_LEN);
  creds.count++;
  return saveWiFiCredentials(creds);
}

// ===== REMOVE WIFI NETWORK BY INDEX =====
bool removeWiFiNetwork(WiFiCredentials& creds, int index) {
  if (index < 0 || index >= creds.count) return false;

  for (int i = index; i < creds.count - 1; i++) {
    creds.networks[i] = creds.networks[i + 1];
  }
  creds.count--;
  return saveWiFiCredentials(creds);
}

// ===== SEED FROM SECRETS (first flash only) =====
// Called on first boot to populate LittleFS from compiled-in secrets
// so the device owner doesn't have to go through the portal
#ifdef SEED_FROM_SECRETS
void seedFromSecrets(DeviceConfig& cfg, WiFiCredentials& creds) {
  if (cfg.configured) return;  // already set up, don't overwrite

  // Populate WiFi from secrets.h arrays
  for (int i = 0; i < WIFI_COUNT && i < MAX_WIFI_NETWORKS; i++) {
    strlcpy(creds.networks[i].ssid, WIFI_SSIDS[i], MAX_SSID_LEN);
    strlcpy(creds.networks[i].pass, WIFI_PASSWORDS[i], MAX_PASS_LEN);
    creds.count++;
  }
  saveWiFiCredentials(creds);

  // Populate device config from secrets.h
  strlcpy(cfg.deviceName,  DEVICE_NAME,  sizeof(cfg.deviceName));
  strlcpy(cfg.aioUsername, AIO_USERNAME, sizeof(cfg.aioUsername));
  strlcpy(cfg.aioKey,      AIO_KEY,      sizeof(cfg.aioKey));
  cfg.configured = true;
  saveConfig(cfg);

  Serial.println("Seeded config from secrets.h");
}
#endif