#pragma once

#include <NimBLEDevice.h>
#include <LittleFS.h>
#include "config.h"
#include "storage.h"
#include "esp_task_wdt.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"

// ===== OTA CHUNK BUFFER =====
// Callback stores chunk here; main loop writes to partition and requests next
static uint8_t  bleOtaChunkBuf[512];
static size_t   bleOtaChunkLen   = 0;
static volatile bool bleOtaChunkReady = false;

static void bleOtaDataCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* data, size_t length, bool isNotify) {
  if (length == 0 || length > sizeof(bleOtaChunkBuf)) return;
  memcpy(bleOtaChunkBuf, data, length);
  bleOtaChunkLen   = length;
  bleOtaChunkReady = true;
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
// VoltMon pulls chunks from collector one at a time
// Each chunk is written to VoltMon's inactive partition before requesting next
// This guarantees: no drops, collector can't outrun VoltMon
static void doBLEOtaTransfer(size_t firmwareSize) {
  Serial.println("BLE OTA: Connecting...");

  // Get inactive OTA partition
  const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
  if (!partition) {
    Serial.println("BLE OTA: No inactive partition");
    return;
  }
  if (firmwareSize > partition->size) {
    Serial.println("BLE OTA: Firmware too large");
    return;
  }
  Serial.println("BLE OTA: Writing to partition: " + String(partition->label));

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);
  NimBLEScanResults results = scan->start(5, false);

  NimBLEAdvertisedDevice* collector = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    if (String(device.getName().c_str()) == "VoltMon-Collector") {
      collector = new NimBLEAdvertisedDevice(device);
      break;
    }
  }
  scan->clearResults();

  if (!collector) { Serial.println("BLE OTA: Collector not found"); return; }

  NimBLEClient* client = NimBLEDevice::createClient();
  bool connected = client->connect(collector);
  delete collector; collector = nullptr;

  if (!connected) {
    Serial.println("BLE OTA: Connection failed");
    NimBLEDevice::deleteClient(client);
    return;
  }
  Serial.println("BLE OTA: Connected");

  NimBLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) { client->disconnect(); NimBLEDevice::deleteClient(client); return; }

  NimBLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);
  NimBLERemoteCharacteristic* otaDataChar  = service->getCharacteristic(BLE_OTA_CHAR_UUID);

  if (!deviceIdChar || !otaDataChar) {
    Serial.println("BLE OTA: Characteristics not found");
    client->disconnect(); NimBLEDevice::deleteClient(client); return;
  }

  client->setConnectionParams(12, 12, 0, 600);

  // Erase partition before writing
  Serial.println("BLE OTA: Erasing partition...");
  esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);
  if (err != ESP_OK) {
    Serial.println("BLE OTA: Erase failed — " + String(esp_err_to_name(err)));
    client->disconnect(); NimBLEDevice::deleteClient(client); return;
  }

  // Subscribe — indication preferred, notification fallback
  bleOtaChunkReady = false;
  bleOtaChunkLen   = 0;
  bool subOk = false;
  if (otaDataChar->canIndicate()) {
    subOk = otaDataChar->subscribe(false, bleOtaDataCallback);
    Serial.println("BLE OTA: Subscribed (indication)");
  } else if (otaDataChar->canNotify()) {
    subOk = otaDataChar->subscribe(true, bleOtaDataCallback);
    Serial.println("BLE OTA: Subscribed (notification)");
  }
  if (!subOk) {
    Serial.println("BLE OTA: Subscribe failed");
    client->disconnect(); NimBLEDevice::deleteClient(client); return;
  }

  // Signal ready and request first chunk
  deviceIdChar->writeValue("OTA_READY", true);
  deviceIdChar->writeValue("NEXT", true);
  Serial.println("BLE OTA: Pulling firmware...");

  size_t         received   = 0;
  uint32_t       offset     = 0;
  unsigned long  otaStart   = millis();
  unsigned long  lastLog    = millis();
  unsigned long  lastWdog   = millis();
  unsigned long  chunkStart = millis();

  while (received < firmwareSize &&
         millis() - otaStart < 360000UL &&
         client->isConnected()) {

    if (millis() - lastWdog > 5000) { esp_task_wdt_reset(); lastWdog = millis(); }

    if (bleOtaChunkReady) {
      bleOtaChunkReady = false;
      size_t len       = bleOtaChunkLen;
      chunkStart       = millis();

      // Write chunk to flash BEFORE requesting next — guarantees order
      err = esp_partition_write(partition, offset, bleOtaChunkBuf, len);
      if (err != ESP_OK) {
        Serial.println("BLE OTA: Write failed at " + String(offset));
        break;
      }
      offset   += len;
      received += len;

      // Now request next chunk
      if (received < firmwareSize) {
        deviceIdChar->writeValue("NEXT", true);
      }
    }

    // Retry if chunk not arriving
    if (!bleOtaChunkReady && millis() - chunkStart > 5000 && received < firmwareSize) {
      Serial.println("BLE OTA: Chunk timeout — retrying NEXT at " + String(received));
      deviceIdChar->writeValue("NEXT", true);
      chunkStart = millis();
    }

    if (millis() - lastLog > 10000) {
      Serial.println("BLE OTA: " +
                     String(received * 100 / firmwareSize) + "% (" +
                     String(received) + "/" + String(firmwareSize) + ")");
      lastLog = millis();
    }
    delay(1);
  }

  client->disconnect();
  NimBLEDevice::deleteClient(client);

  if (received < firmwareSize) {
    Serial.println("BLE OTA: Failed — received " + String(received) +
                   "/" + String(firmwareSize));
    return;
  }

  Serial.println("BLE OTA: All bytes written — setting boot partition");
  err = esp_ota_set_boot_partition(partition);
  if (err != ESP_OK) {
    Serial.println("BLE OTA: Set boot failed — " + String(esp_err_to_name(err)));
    return;
  }
  Serial.println("BLE OTA: Success — rebooting");
  NimBLEDevice::deinit(true);
  delay(500);
  esp_restart();
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
      std::string mfData = device.getManufacturerData();
      if (mfData.length() >= 9) {
        uint8_t id0 = (uint8_t)mfData[0];
        uint8_t id1 = (uint8_t)mfData[1];
        if (id0 == 0xFF && id1 == 0xFF) {
          uint8_t maj = (uint8_t)mfData[2];
          uint8_t min = (uint8_t)mfData[3];
          uint8_t pat = (uint8_t)mfData[4];
          uint32_t sz = 0;
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

  if (!collector) { Serial.println("BLE: Collector not found"); return false; }

  NimBLEClient* client = NimBLEDevice::createClient();
  bool connected = client->connect(collector);
  delete collector; collector = nullptr;

  if (!connected) {
    Serial.println("BLE: Connection failed");
    NimBLEDevice::deleteClient(client);
    return false;
  }
  Serial.println("BLE: Connected to collector");

  NimBLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) {
    client->disconnect(); NimBLEDevice::deleteClient(client); return false;
  }

  NimBLERemoteCharacteristic* recordsChar  = service->getCharacteristic(BLE_RECORDS_CHAR_UUID);
  NimBLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);

  if (!recordsChar || !deviceIdChar) {
    Serial.println("BLE: Required characteristics not found");
    client->disconnect(); NimBLEDevice::deleteClient(client); return false;
  }

  deviceIdChar->writeValue((String(cfg.deviceName) + ":" + String(VOLTMON_VERSION)).c_str(), true);

  bool transferOk = false;
  if (hasRecords) {
    String records = loadPendingRecords();
    if (records.length() > 0) {
      recordsChar->writeValue((String(cfg.deviceName) + "|" + records).c_str(), true);
      Serial.println("BLE: Sent " + String(records.length()) + " bytes");
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
    Serial.println("BLE OTA: Starting OTA pull...");
    delay(500);
    doBLEOtaTransfer(otaSize);
  }

  return transferOk;
}