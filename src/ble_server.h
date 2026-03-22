#pragma once

#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <Update.h>
#include "config.h"
#include "storage.h"
#include "esp_task_wdt.h"

// ===== OTA RECEIVE STATE =====
static volatile size_t bleOtaReceived    = 0;
static volatile bool   bleOtaChunkReady  = false;
static uint8_t         bleOtaChunkBuf[512];
static size_t          bleOtaChunkLen    = 0;

static void bleOtaDataCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* data, size_t length, bool isNotify) {
  if (length == 0) return;
  // Copy chunk to buffer for main task to process
  if (length <= sizeof(bleOtaChunkBuf)) {
    memcpy(bleOtaChunkBuf, data, length);
    bleOtaChunkLen   = length;
    bleOtaChunkReady = true;
  }
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

// ===== DEDICATED OTA CONNECTION — PULL MODEL =====
// VoltMon requests each chunk by writing "NEXT" to deviceIdChar
// Collector sends one chunk per request — guaranteed delivery, no drops
static void doBLEOtaTransfer(size_t firmwareSize) {
  Serial.println("BLE OTA: Connecting...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);

  NimBLEScanResults results = scan->start(5, false);

  NimBLEAdvertisedDevice* collector = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    String name = String(device.getName().c_str());
    if (name == "VoltMon-Collector") {
      collector = new NimBLEAdvertisedDevice(device);
      break;
    }
  }
  scan->clearResults();

  if (!collector) {
    Serial.println("BLE OTA: Collector not found");
    return;
  }

  NimBLEClient* client = NimBLEDevice::createClient();
  bool connected = client->connect(collector);
  delete collector;
  collector = nullptr;

  if (!connected) {
    Serial.println("BLE OTA: Connection failed");
    NimBLEDevice::deleteClient(client);
    return;
  }

  Serial.println("BLE OTA: Connected");

  NimBLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) {
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return;
  }

  NimBLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);
  NimBLERemoteCharacteristic* otaDataChar  = service->getCharacteristic(BLE_OTA_CHAR_UUID);

  if (!deviceIdChar || !otaDataChar) {
    Serial.println("BLE OTA: Characteristics not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return;
  }

  client->setConnectionParams(12, 12, 0, 600);

  if (!Update.begin(firmwareSize)) {
    Serial.println("BLE OTA: Update.begin failed");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return;
  }

  bleOtaReceived   = 0;
  bleOtaChunkReady = false;

  if (!otaDataChar->canNotify()) {
    Serial.println("BLE OTA: Cannot subscribe");
    Update.abort();
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return;
  }

  otaDataChar->subscribe(true, bleOtaDataCallback);
  Serial.println("BLE OTA: Subscribed");

  // Signal OTA_READY — collector enters pull mode waiting for NEXT requests
  deviceIdChar->writeValue("OTA_READY", true);
  Serial.println("BLE OTA: Sent OTA_READY — pulling firmware...");

  unsigned long otaStart   = millis();
  unsigned long lastLog    = millis();
  unsigned long lastWdog   = millis();
  unsigned long chunkStart = millis();

  // Request first chunk
  deviceIdChar->writeValue("NEXT", true);

  while (bleOtaReceived < firmwareSize &&
         millis() - otaStart < 360000UL &&
         client->isConnected()) {

    if (millis() - lastWdog > 5000) {
      esp_task_wdt_reset();
      lastWdog = millis();
    }

    if (bleOtaChunkReady) {
      size_t chunkLen  = bleOtaChunkLen;
      bleOtaChunkReady = false;
      chunkStart       = millis();

      // Request next chunk BEFORE writing to flash — pipeline overlap
      // Collector prepares next chunk while VoltMon writes current one
      size_t projectedReceived = bleOtaReceived + chunkLen;
      if (projectedReceived < firmwareSize) {
        deviceIdChar->writeValue("NEXT", true);
      }

      // Now write to flash
      Update.write(bleOtaChunkBuf, chunkLen);
      bleOtaReceived += chunkLen;
    }

    // Timeout waiting for chunk
    if (!bleOtaChunkReady && millis() - chunkStart > 5000 && bleOtaReceived < firmwareSize) {
      Serial.println("BLE OTA: Chunk timeout at " + String(bleOtaReceived) + " — retrying");
      deviceIdChar->writeValue("NEXT", true);
      chunkStart = millis();
    }

    if (millis() - lastLog > 10000) {
      Serial.println("BLE OTA: " +
                     String(bleOtaReceived * 100 / firmwareSize) + "% (" +
                     String(bleOtaReceived) + "/" + String(firmwareSize) + ")");
      lastLog = millis();
    }

    delay(1);
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
      Serial.println("BLE OTA: Update.end failed — " + String(Update.errorString()));
      Update.abort();
    }
  } else {
    Serial.println("BLE OTA: Failed — received " +
                   String(bleOtaReceived) + "/" + String(firmwareSize));
    Update.abort();
  }

  client->disconnect();
  NimBLEDevice::deleteClient(client);
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
  size_t otaSize = 0;

  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    String name = String(device.getName().c_str());

    if (name == "VoltMon-Collector") {
      collector = new NimBLEAdvertisedDevice(device);

      // Read OTA version+size from manufacturer data
      std::string mfData = device.getManufacturerData();
      if (mfData.length() >= 9) {
        uint8_t id0 = (uint8_t)mfData[0];
        uint8_t id1 = (uint8_t)mfData[1];
        if (id0 == 0xFF && id1 == 0xFF) {
          uint8_t  maj = (uint8_t)mfData[2];
          uint8_t  min = (uint8_t)mfData[3];
          uint8_t  pat = (uint8_t)mfData[4];
          uint32_t sz  = 0;
          memcpy(&sz, &mfData[5], 4);
          char advVer[16];
          snprintf(advVer, sizeof(advVer), "%d.%d.%d", maj, min, pat);
          Serial.println("BLE: Advert OTA version: " + String(advVer) +
                         " ours: " + String(VOLTMON_VERSION));
          if (String(advVer) != String(VOLTMON_VERSION) && sz > 0) {
            otaSize = (size_t)sz;
            Serial.println("BLE: OTA needed — size: " + String(otaSize));
          } else {
            Serial.println("BLE: OTA not needed — already on " + String(VOLTMON_VERSION));
          }
        }
      }
      Serial.println("BLE: Collector found" +
                     String(otaSize > 0 ? " (OTA available)" : ""));
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
    Serial.println("BLE: Required characteristics not found!");
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

  client->disconnect();
  NimBLEDevice::deleteClient(client);
  Serial.println("BLE: Transfer done — success: " + String(transferOk));

  if (otaSize > 0) {
    Serial.println("BLE OTA: Starting dedicated OTA connection...");
    delay(500);
    doBLEOtaTransfer(otaSize);
  }

  return transferOk;
}