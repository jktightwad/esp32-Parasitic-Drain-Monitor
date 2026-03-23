#pragma once

#include <NimBLEDevice.h>
#include <LittleFS.h>
#include "config.h"
#include "storage.h"
#include "esp_task_wdt.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"

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

// ===== OTA SERVER MODE =====
// VoltMon becomes a BLE SERVER advertising "VoltMon-OTA"
// Collector connects as CLIENT and writes firmware chunks directly
// Server onWrite callback = guaranteed delivery, no queue overflow

static volatile size_t   otaServerReceived = 0;
static volatile bool     otaServerDone     = false;
static const esp_partition_t* otaPartition = nullptr;
static uint32_t              otaWriteOffset = 0;

// Page buffer for efficient flash writes
static const size_t OTA_PAGE_SIZE = 4096;
static uint8_t      otaPageBuf[OTA_PAGE_SIZE];
static size_t       otaPageUsed = 0;

static void flushOtaPage(bool force = false) {
  if (otaPageUsed == 0) return;
  if (!force && otaPageUsed < OTA_PAGE_SIZE) return;
  if (otaPartition) {
    esp_partition_write(otaPartition, otaWriteOffset, otaPageBuf, otaPageUsed);
    otaWriteOffset += otaPageUsed;
  }
  otaPageUsed = 0;
}

class OtaWriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    size_t len = val.length();
    if (len == 0) return;

    // Check for end marker
    if (len == 3 && memcmp(val.data(), "END", 3) == 0) {
      flushOtaPage(true);  // flush remaining bytes
      otaServerDone = true;
      return;
    }

    // Accumulate into page buffer
    const uint8_t* data = (const uint8_t*)val.data();
    size_t remaining = len;
    size_t dataOffset = 0;

    while (remaining > 0) {
      size_t space = OTA_PAGE_SIZE - otaPageUsed;
      size_t toCopy = min(remaining, space);
      memcpy(otaPageBuf + otaPageUsed, data + dataOffset, toCopy);
      otaPageUsed    += toCopy;
      dataOffset     += toCopy;
      remaining      -= toCopy;
      otaServerReceived += toCopy;

      if (otaPageUsed >= OTA_PAGE_SIZE) {
        flushOtaPage(true);
      }
    }
  }
};

static bool doOtaServerMode(size_t firmwareSize) {
  Serial.println("BLE OTA: Server mode — advertising VoltMon-OTA");

  otaPartition = esp_ota_get_next_update_partition(NULL);
  if (!otaPartition) {
    Serial.println("BLE OTA: No inactive partition");
    return false;
  }
  if (firmwareSize > otaPartition->size) {
    Serial.println("BLE OTA: Firmware too large");
    return false;
  }

  // Erase partition
  esp_task_wdt_deinit();
  esp_err_t err = esp_partition_erase_range(otaPartition, 0, otaPartition->size);
  esp_task_wdt_init(120, false);
  if (err != ESP_OK) {
    Serial.println("BLE OTA: Erase failed");
    return false;
  }

  otaServerReceived = 0;
  otaServerDone     = false;
  otaWriteOffset    = 0;
  otaPageUsed       = 0;

  // Start BLE server advertising "VoltMon-OTA"
  NimBLEDevice::init("VoltMon-OTA");
  NimBLEDevice::setMTU(512);

  NimBLEServer*  server  = NimBLEDevice::createServer();
  NimBLEService* service = server->createService(BLE_SERVICE_UUID);

  // OTA data char — WRITE_NR for maximum throughput
  NimBLECharacteristic* otaChar = service->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE_NR
  );
  otaChar->setCallbacks(new OtaWriteCallbacks());

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("BLE OTA: Waiting for collector to connect and push firmware...");

  unsigned long otaStart = millis();
  unsigned long lastLog  = millis();
  unsigned long lastWdog = millis();

  while (!otaServerDone &&
         millis() - otaStart < 360000UL) {
    if (millis() - lastWdog > 5000) {
      esp_task_wdt_reset();
      lastWdog = millis();
    }
    if (millis() - lastLog > 10000) {
      Serial.println("BLE OTA: " +
                     String(otaServerReceived * 100 / firmwareSize) + "% (" +
                     String(otaServerReceived) + "/" + String(firmwareSize) + ")");
      lastLog = millis();
    }
    delay(10);
  }

  NimBLEDevice::deinit(true);

  if (!otaServerDone || otaServerReceived < firmwareSize) {
    Serial.println("BLE OTA: Failed — received " +
                   String(otaServerReceived) + "/" + String(firmwareSize));
    return false;
  }

  Serial.println("BLE OTA: All bytes received — setting boot partition");
  err = esp_ota_set_boot_partition(otaPartition);
  if (err != ESP_OK) {
    Serial.println("BLE OTA: Set boot failed");
    return false;
  }

  Serial.println("BLE OTA: Success — rebooting");
  delay(500);
  esp_restart();
  return true;  // never reached
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

  // OTA: VoltMon becomes server, collector connects and pushes firmware
  if (otaSize > 0) {
    Serial.println("BLE OTA: Switching to server mode for firmware receive...");
    delay(500);
    doOtaServerMode(otaSize);
  }

  return transferOk;
}