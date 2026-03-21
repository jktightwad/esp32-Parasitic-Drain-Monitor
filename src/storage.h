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
#define MAX_AIO_LEN        64

// ===== DEVICE CONFIG =====
struct DeviceConfig {
  char deviceName[MAX_DEVICE_NAME] = "";
  char aioUsername[MAX_AIO_LEN]    = "";
  char aioKey[MAX_AIO_LEN]         = "";
  bool configured                  = false;
  bool debugMode                   = false;
  int  sleepSeconds                = 300;
};

// ===== WIFI CREDENTIAL ENTRY =====
struct WiFiCredential {
  String ssid = "";
  String pass = "";
};

// ===== WIFI CREDENTIAL LIST =====
struct WiFiCredentials {
  WiFiCredential networks[MAX_WIFI_NETWORKS];
  int count = 0;
};

// ===== LOAD DEVICE CONFIG =====
inline bool loadConfig(DeviceConfig& cfg) {
  if (!LittleFS.exists(CONFIG_FILE)) return false;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) return false;

  strlcpy(cfg.deviceName,  doc["deviceName"]  | "", sizeof(cfg.deviceName));
  strlcpy(cfg.aioUsername, doc["aioUsername"] | "", sizeof(cfg.aioUsername));
  strlcpy(cfg.aioKey,      doc["aioKey"]      | "", sizeof(cfg.aioKey));
  cfg.configured   = doc["configured"]   | false;
  cfg.debugMode    = doc["debugMode"]    | false;
  cfg.sleepSeconds = doc["sleepSeconds"] | 300;

  return true;
}

// ===== SAVE DEVICE CONFIG =====
inline bool saveConfig(const DeviceConfig& cfg) {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return false;

  JsonDocument doc;
  doc["deviceName"]  = cfg.deviceName;
  doc["aioUsername"] = cfg.aioUsername;
  doc["aioKey"]      = cfg.aioKey;
  doc["configured"]  = cfg.configured;
  doc["debugMode"]   = cfg.debugMode;
  doc["sleepSeconds"] = cfg.sleepSeconds;

  serializeJson(doc, f);
  f.close();
  return true;
}

// ===== LOAD WIFI CREDENTIALS =====
inline bool loadWiFiCredentials(WiFiCredentials& creds) {
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
    creds.networks[creds.count].ssid = net["ssid"] | "";
    creds.networks[creds.count].pass = net["pass"] | "";
    creds.count++;
  }

  return creds.count > 0;
}

// ===== SAVE WIFI CREDENTIALS =====
inline bool saveWiFiCredentials(const WiFiCredentials& creds) {
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
inline bool addWiFiNetwork(WiFiCredentials& creds, const char* ssid, const char* pass) {
  if (creds.count >= MAX_WIFI_NETWORKS) return false;

  for (int i = 0; i < creds.count; i++) {
    if (creds.networks[i].ssid == String(ssid)) {
      creds.networks[i].pass = String(pass);
      return saveWiFiCredentials(creds);
    }
  }

  creds.networks[creds.count].ssid = String(ssid);
  creds.networks[creds.count].pass = String(pass);
  creds.count++;
  return saveWiFiCredentials(creds);
}

// ===== REMOVE WIFI NETWORK BY INDEX =====
inline bool removeWiFiNetwork(WiFiCredentials& creds, int index) {
  if (index < 0 || index >= creds.count) return false;

  for (int i = index; i < creds.count - 1; i++) {
    creds.networks[i] = creds.networks[i + 1];
  }
  creds.count--;
  return saveWiFiCredentials(creds);
}

// ===== SEED FROM SECRETS (first flash only) =====
#ifdef SEED_FROM_SECRETS
inline void seedFromSecrets(DeviceConfig& cfg, WiFiCredentials& creds) {
  if (cfg.configured) return;

  for (int i = 0; i < WIFI_COUNT && i < MAX_WIFI_NETWORKS; i++) {
    creds.networks[i].ssid = String(WIFI_SSIDS[i]);
    creds.networks[i].pass = String(WIFI_PASSWORDS[i]);
    creds.count++;
  }
  saveWiFiCredentials(creds);

  strlcpy(cfg.deviceName,  DEVICE_NAME,  sizeof(cfg.deviceName));
  strlcpy(cfg.aioUsername, AIO_USERNAME, sizeof(cfg.aioUsername));
  strlcpy(cfg.aioKey,      AIO_KEY,      sizeof(cfg.aioKey));
  cfg.configured   = true;
  cfg.debugMode    = false;
  cfg.sleepSeconds = 300;
  saveConfig(cfg);

  Serial.println("Seeded config from secrets.h");
}
#endif