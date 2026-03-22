#pragma once

#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <Update.h>
#include "config.h"
#include "storage.h"
#include "esp_task_wdt.h"

// ===== OTA RECEIVE STATE =====
static volatile size_t bleOtaReceived = 0;

static void bleOtaDataCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* data, size_t length, bool isNotify) {
  if (length == 0) return;
  if (length == 1 && data[0] == 0x00) return;  // keep-alive
  Update.write(data, length);
  bleOtaReceived += length;
}

// ===== LOAD PENDING RECORDS =====
static String loadPendingRecords() {
  if (!LittleFS.exists(PENDING_FILE)) return "";
  File f = LittleFS.open(PENDING_FILE, "r");
  if (!f) return "";
  String content = "";
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      if (content.length() > 0) content += "|";
      content += line;
    }
  }
  f.close();
  return content;
}

// ===== OTA CONNECTION — dedicated second connection for firmware transfer =====
static bool doBLEOtaTransfer(size_t firmwareSize) {
  Serial.println("BLE OTA: Scanning for collector...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);

  NimBLEScanResults results = scan->start(5, false);

  NimBLEAdvertisedDevice* collector = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    String name = String(device.getName().c_str());
    if (name.startsWith("VoltMon-Collector")) {
      collector = new NimBLEAdvertisedDevice(device);
      Serial.println("BLE OTA: Collector found: " + name);
      break;
    }
  }
  scan->clearResults();

  if (!collector) {
    Serial.println("BLE OTA: Collector not found");
    return false;
  }

  NimBLEClient* client = NimBLEDevice::createClient();
  bool connected = client->connect(collector);
  delete collector;
  collector = nullptr;

  if (!connected) {
    Serial.println("BLE OTA: Connection failed");
    NimBLEDevice::deleteClient(client);
    return false;
  }

  Serial.println("BLE OTA: Connected");

  NimBLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) {
    Serial.println("BLE OTA: Service not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);
  NimBLERemoteCharacteristic* otaDataChar  = service->getCharacteristic(BLE_OTA_CHAR_UUID);

  if (!deviceIdChar || !otaDataChar) {
    Serial.println("BLE OTA: Characteristics not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  // Extended supervision timeout for OTA
  client->setConnectionParams(12, 12, 0, 600);

  if (!Update.begin(firmwareSize)) {
    Serial.println("BLE OTA: Update.begin failed");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  bleOtaReceived = 0;

  if (!otaDataChar->canNotify()) {
    Serial.println("BLE OTA: Cannot subscribe to data");
    Update.abort();
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  otaDataChar->subscribe(true, bleOtaDataCallback);
  Serial.println("BLE OTA: Subscribed to data");

  // Signal ready — collector sees this in DeviceIdCallbacks and starts streaming
  deviceIdChar->writeValue("OTA_READY", true);
  Serial.println("BLE OTA: Sent OTA_READY — waiting for firmware...");

  unsigned long otaStart = millis();
  unsigned long lastLog  = millis();
  unsigned long lastWdog = millis();

  while (bleOtaReceived < firmwareSize &&
         millis() - otaStart < 360000UL &&
         client->isConnected()) {
    if (millis() - lastWdog > 5000) {
      esp_task_wdt_reset();
      lastWdog = millis();
    }
    if (millis() - lastLog > 10000) {
      Serial.println("BLE OTA: " +
                     String(bleOtaReceived * 100 / firmwareSize) + "% (" +
                     String(bleOtaReceived) + "/" + String(firmwareSize) + ")");
      lastLog = millis();
    }
    delay(10);
  }

  if (bleOtaReceived >= firmwareSize) {
    if (Update.end(true)) {
      Serial.println("BLE OTA: Success — rebooting");
      client->disconnect();
      NimBLEDevice::deleteClient(client);
      NimBLEDevice::deinit(true);
      delay(500);
      ESP.restart();
    } else {
      Serial.println("BLE OTA: Update.end failed");
      Update.abort();
    }
  } else {
    Serial.println("BLE OTA: Timeout — received " +
                   String(bleOtaReceived) + "/" + String(firmwareSize));
    Update.abort();
  }

  client->disconnect();
  NimBLEDevice::deleteClient(client);
  return false;
}

// ===== SCAN FOR COLLECTOR AND TRANSFER =====
bool bleScanAndTransfer(DeviceConfig& cfg, bool hasRecords) {
  Serial.println("BLE: scanning for collector...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);

  NimBLEScanResults results = scan->start(5, false);

  NimBLEAdvertisedDevice* collector = nullptr;
  String otaVersion = "";
  size_t otaSize    = 0;

  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    String name = String(device.getName().c_str());

    if (name.startsWith("VoltMon-Collector")) {
      collector = new NimBLEAdvertisedDevice(device);
      Serial.println("BLE: Collector found: " + name);

      // Check for OTA info in name: "VoltMon-Collector:2.1.9:1380080"
      int firstColon = name.indexOf(':');
      if (firstColon > 0) {
        int secondColon = name.indexOf(':', firstColon + 1);
        if (secondColon > 0) {
          otaVersion = name.substring(firstColon + 1, secondColon);
          otaSize    = name.substring(secondColon + 1).toInt();
          Serial.println("BLE: OTA available in advert: v" + otaVersion +
                         " (" + String(otaSize) + " bytes)");
        }
      }
      break;
    }
  }
  scan->clearResults();

  if (!collector) {
    Serial.println("BLE: Collector not found");
    return false;
  }

  NimBLEClient* client = NimBLEDevice::createClient();
  bool connected = client->connect(collector);
  delete collector;
  collector = nullptr;

  if (!connected) {
    Serial.println("BLE: Connection failed");
    NimBLEDevice::deleteClient(client);
    return false;
  }

  Serial.println("BLE: Connected to collector");

  NimBLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) {
    Serial.println("BLE: Service not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteCharacteristic* recordsChar  = service->getCharacteristic(BLE_RECORDS_CHAR_UUID);
  NimBLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);

  if (!recordsChar || !deviceIdChar) {
    Serial.println("BLE: Required characteristics not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  // Send device ID + version
  String deviceIdPayload = String(cfg.deviceName) + ":" + String(VOLTMON_VERSION);
  deviceIdChar->writeValue(deviceIdPayload.c_str(), true);

  bool transferOk = false;

  if (hasRecords) {
    String records = loadPendingRecords();
    if (records.length() > 0) {
      String payload = String(cfg.deviceName) + "|" + records;
      recordsChar->writeValue(payload.c_str(), true);
      Serial.println("BLE: Sent " + String(payload.length()) + " bytes");
      transferOk = true;
      Serial.println("BLE: Transfer accepted");
    } else {
      recordsChar->writeValue("EMPTY", true);
      transferOk = true;
    }
  } else {
    recordsChar->writeValue("EMPTY", true);
    transferOk = true;
  }

  // Disconnect records connection cleanly
  client->disconnect();
  NimBLEDevice::deleteClient(client);
  Serial.println("BLE: Transfer done — success: " + String(transferOk));

  // OTA available if advertised and version differs from ours
  if (otaSize > 0 && otaVersion.length() > 0 &&
      otaVersion != String(VOLTMON_VERSION)) {
    Serial.println("BLE OTA: Version mismatch detected — " +
                   String(VOLTMON_VERSION) + " -> " + otaVersion +
                   ", starting OTA connection...");
    delay(500);
    doBLEOtaTransfer(otaSize);
  }

  return transferOk;
}