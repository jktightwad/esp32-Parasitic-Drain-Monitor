#pragma once

#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <Update.h>
#include "config.h"
#include "storage.h"
#include "esp_task_wdt.h"

// ===== OTA RECEIVE STATE =====
static volatile size_t bleOtaReceived  = 0;
static volatile bool   bleOtaDone      = false;
static volatile bool   bleOtaSuccess   = false;
static volatile String bleOtaCtrlValue = "";

static void bleOtaDataCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* data, size_t length, bool isNotify) {
  if (length == 0) return;
  // Single 0x00 byte is a keep-alive — ignore
  if (length == 1 && data[0] == 0x00) return;
  Update.write(data, length);
  bleOtaReceived += length;
}

static void bleOtaCtrlCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* data, size_t length, bool isNotify) {
  if (length == 0) return;
  bleOtaCtrlValue = String((char*)data).substring(0, length);
  bleOtaCtrlValue.trim();
  Serial.println("BLE OTA ctrl notify: [" + bleOtaCtrlValue + "]");
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

// ===== SCAN FOR COLLECTOR AND TRANSFER =====
bool bleScanAndTransfer(DeviceConfig& cfg, bool hasRecords) {
  Serial.println("BLE: scanning for collector...");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(90);

  NimBLEScanResults results = scan->start(5, false);

  NimBLEAdvertisedDevice* collector = nullptr;
  for (int i = 0; i < results.getCount(); i++) {
    NimBLEAdvertisedDevice device = results.getDevice(i);
    if (device.getName() == "VoltMon-Collector") {
      collector = new NimBLEAdvertisedDevice(device);
      Serial.println("BLE: Collector found");
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
  NimBLERemoteCharacteristic* confirmChar  = service->getCharacteristic(BLE_CONFIRM_CHAR_UUID);
  NimBLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);
  NimBLERemoteCharacteristic* otaCtrlChar  = service->getCharacteristic(BLE_OTA_CTRL_CHAR_UUID);
  NimBLERemoteCharacteristic* otaDataChar  = service->getCharacteristic(BLE_OTA_CHAR_UUID);

  if (!recordsChar || !confirmChar || !deviceIdChar) {
    Serial.println("BLE: Required characteristics not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  // Subscribe to OTA ctrl notifications BEFORE writing device ID
  // so we don't miss the notification from the collector
  bleOtaCtrlValue = "";
  if (otaCtrlChar && otaCtrlChar->canNotify()) {
    otaCtrlChar->subscribe(true, bleOtaCtrlCallback);
    Serial.println("BLE: Subscribed to OTA ctrl notifications");
  }

  // Send device ID + current version so collector can check for OTA
  String deviceIdPayload = String(cfg.deviceName) + ":" + String(VOLTMON_VERSION);
  deviceIdChar->writeValue(deviceIdPayload.c_str(), true);

  bool transferOk = false;

  if (hasRecords) {
    String records = loadPendingRecords();
    if (records.length() > 0) {
      String payload = String(cfg.deviceName) + "|" + records;
      recordsChar->writeValue(payload.c_str(), true);
      Serial.println("BLE: Sent " + String(payload.length()) + " bytes");

      unsigned long start = millis();
      while (millis() - start < 10000) {
        std::string confirm = confirmChar->readValue();
        String cs = String(confirm.c_str());
        cs.trim();
        if (cs == "OK") {
          transferOk = true;
          Serial.println("BLE: Confirmed OK");
          break;
        } else if (cs == "FAIL") {
          Serial.println("BLE: Confirmed FAIL");
          break;
        }
        delay(200);
      }
    } else {
      recordsChar->writeValue("EMPTY", true);
      transferOk = true;
    }
  } else {
    recordsChar->writeValue("EMPTY", true);
    transferOk = true;
  }

  // ===== CHECK FOR OTA =====
  // Wait up to 3 seconds for collector to notify OTA command
  if (otaCtrlChar && otaDataChar) {
    Serial.println("BLE: Waiting for OTA ctrl notification...");
    unsigned long ctrlWait = millis();
    while (bleOtaCtrlValue.length() == 0 &&
           millis() - ctrlWait < 3000 &&
           client->isConnected()) {
      delay(50);
    }

    Serial.println("BLE OTA ctrl: [" + bleOtaCtrlValue + "]");

    if (bleOtaCtrlValue.startsWith("OTA_START:")) {
      size_t firmwareSize = bleOtaCtrlValue.substring(10).toInt();
      Serial.println("BLE OTA: Starting — " + String(firmwareSize) + " bytes");

      if (firmwareSize == 0 || !Update.begin(firmwareSize)) {
        Serial.println("BLE OTA: Update.begin failed");
      } else {
        // Request longer supervision timeout for OTA (6000ms = 600 * 10ms)
        client->setConnectionParams(12, 12, 0, 600);
        Serial.println("BLE OTA: Extended connection timeout");

        bleOtaReceived = 0;

        // Subscribe to OTA data notifications
        bool otaSubscribed = false;
        if (otaDataChar->canNotify()) {
          otaDataChar->subscribe(true, bleOtaDataCallback);
          otaSubscribed = true;
          Serial.println("BLE OTA: Subscribed to data");
          // Signal collector we are ready
          deviceIdChar->writeValue("OTA_READY", true);
          Serial.println("BLE OTA: Sent OTA_READY");
        } else {
          Serial.println("BLE OTA: Cannot subscribe");
          Update.abort();
        }

        if (otaSubscribed) {
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
        }
      }
    }
  }

  client->disconnect();
  NimBLEDevice::deleteClient(client);

  Serial.println("BLE: Transfer done — success: " + String(transferOk));
  return transferOk;
}