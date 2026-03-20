#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include "config.h"
#include "storage.h"

// ===== STATE =====
static BLEServer*          bleServer          = nullptr;
static BLECharacteristic*  bleRecordsChar     = nullptr;
static BLECharacteristic*  bleConfirmChar     = nullptr;
static BLECharacteristic*  bleDeviceIdChar    = nullptr;
static BLECharacteristic*  bleOtaChar         = nullptr;
static BLECharacteristic*  bleOtaCtrlChar     = nullptr;

static bool bleConnected       = false;
static bool bleTransferDone    = false;
static bool bleTransferSuccess = false;
static bool bleOtaPending      = false;

static size_t otaExpectedSize  = 0;
static size_t otaReceivedBytes = 0;
static bool   otaInProgress    = false;

// ===== SERVER CALLBACKS =====
class VoltMonServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleConnected       = true;
    bleTransferDone    = false;
    bleTransferSuccess = false;
    Serial.println("BLE: Collector connected");
  }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    Serial.println("BLE: Collector disconnected");
  }
};

// ===== CONFIRM CALLBACKS =====
class ConfirmCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String val = pChar->getValue().c_str();
    val.trim();
    Serial.println("BLE confirm: " + val);
    if (val == "OK") {
      bleTransferSuccess = true;
      bleTransferDone    = true;
    } else if (val == "FAIL") {
      bleTransferSuccess = false;
      bleTransferDone    = true;
    }
  }
};

// ===== OTA CONTROL CALLBACKS =====
class OtaCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String val = pChar->getValue().c_str();
    val.trim();
    Serial.println("BLE OTA ctrl: " + val);

    if (val.startsWith("OTA_START:")) {
      otaExpectedSize  = val.substring(10).toInt();
      otaReceivedBytes = 0;
      otaInProgress    = true;
      if (!Update.begin(otaExpectedSize)) {
        Serial.println("OTA begin failed");
        otaInProgress = false;
      } else {
        Serial.println("OTA started — " + String(otaExpectedSize) + " bytes");
      }
    } else if (val == "OTA_END") {
      if (otaInProgress) {
        if (Update.end(true)) {
          Serial.println("OTA complete — rebooting");
          bleOtaPending = true;
        } else {
          Serial.println("OTA end failed");
          Update.abort();
        }
        otaInProgress = false;
      }
    } else if (val == "OTA_ABORT") {
      Update.abort();
      otaInProgress    = false;
      otaReceivedBytes = 0;
    }
  }
};

// ===== OTA DATA CALLBACKS =====
class OtaDataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    if (!otaInProgress) return;
    std::string data = pChar->getValue();
    size_t len = data.length();
    if (len == 0) return;
    size_t written = Update.write((uint8_t*)data.c_str(), len);
    otaReceivedBytes += written;
    String progress = "ACK:" + String(otaReceivedBytes);
    bleOtaChar->setValue(progress.c_str());
    bleOtaChar->notify();
  }
};

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

// ===== SEND RECORDS IN CHUNKS =====
static void sendRecords(const String& deviceName, const String& records) {
  if (records.length() == 0) {
    bleRecordsChar->setValue("EMPTY");
    bleRecordsChar->notify();
    return;
  }

  String payload    = deviceName + "|" + records;
  const int CHUNK_SIZE = 400;
  int totalChunks   = (payload.length() + CHUNK_SIZE - 1) / CHUNK_SIZE;

  Serial.println("BLE: sending " + String(payload.length()) +
                 " bytes in " + String(totalChunks) + " chunks");

  for (int seq = 0; seq < totalChunks; seq++) {
    if (!bleConnected) return;
    int start  = seq * CHUNK_SIZE;
    int len    = min(CHUNK_SIZE, (int)payload.length() - start);
    String chunk = "CHUNK:" + String(seq) + ":" + String(totalChunks) + ":" +
                   payload.substring(start, start + len);
    bleRecordsChar->setValue(chunk.c_str());
    bleRecordsChar->notify();
    delay(50);
  }

  bleRecordsChar->setValue("TRANSFER_COMPLETE");
  bleRecordsChar->notify();
}

// ===== INIT BLE SERVER =====
void bleServerInit(const DeviceConfig& cfg) {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(512);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new VoltMonServerCallbacks());

  BLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  bleDeviceIdChar = service->createCharacteristic(
    BLE_DEVICE_ID_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleDeviceIdChar->addDescriptor(new BLE2902());
  bleDeviceIdChar->setValue(cfg.deviceName);

  bleRecordsChar = service->createCharacteristic(
    BLE_RECORDS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleRecordsChar->addDescriptor(new BLE2902());
  bleRecordsChar->setValue("");

  bleConfirmChar = service->createCharacteristic(
    BLE_CONFIRM_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  bleConfirmChar->setCallbacks(new ConfirmCallbacks());

  bleOtaChar = service->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  bleOtaChar->addDescriptor(new BLE2902());
  bleOtaChar->setCallbacks(new OtaDataCallbacks());

  bleOtaCtrlChar = service->createCharacteristic(
    BLE_OTA_CTRL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  bleOtaCtrlChar->setCallbacks(new OtaCtrlCallbacks());

  service->start();
  Serial.println("BLE server started — " + String(cfg.deviceName));
}

// ===== SCAN FOR COLLECTOR AND TRANSFER =====
bool bleScanAndTransfer(DeviceConfig& cfg, bool hasRecords) {
  Serial.println("BLE: scanning for collector...");

  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(90);

  BLEScanResults results = scan->start(2, false);

  BLEAdvertisedDevice* collector = nullptr;

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    if (device.getName() == "VoltMon-Collector") {
      collector = new BLEAdvertisedDevice(device);
      Serial.println("BLE: Collector found");
      break;
    }
  }
  scan->clearResults();

  if (!collector) {
    Serial.println("BLE: Collector not found");
    delete collector;
    return false;
  }

  BLEClient* client = BLEDevice::createClient();
  if (!client->connect(collector)) {
    Serial.println("BLE: Connection failed");
    delete collector;
    delete client;
    return false;
  }
  delete collector;

  Serial.println("BLE: Connected to collector");

  BLERemoteService* service = client->getService(BLE_SERVICE_UUID);
  if (!service) {
    Serial.println("BLE: Service not found");
    client->disconnect();
    delete client;
    return false;
  }

  BLERemoteCharacteristic* recordsChar  = service->getCharacteristic(BLE_RECORDS_CHAR_UUID);
  BLERemoteCharacteristic* confirmChar  = service->getCharacteristic(BLE_CONFIRM_CHAR_UUID);
  BLERemoteCharacteristic* deviceIdChar = service->getCharacteristic(BLE_DEVICE_ID_CHAR_UUID);
  BLERemoteCharacteristic* otaCtrlChar  = service->getCharacteristic(BLE_OTA_CTRL_CHAR_UUID);

  if (!recordsChar || !confirmChar || !deviceIdChar) {
    Serial.println("BLE: Required characteristics not found");
    client->disconnect();
    delete client;
    return false;
  }

  // Send device ID
  deviceIdChar->writeValue(cfg.deviceName);

  bool transferOk = false;

  if (hasRecords) {
    String records = loadPendingRecords();
    if (records.length() > 0) {
      recordsChar->writeValue(records.c_str());
      Serial.println("BLE: Sent " + String(records.length()) + " bytes");

      unsigned long start = millis();
      while (millis() - start < 10000) {
        String confirm = confirmChar->readValue().c_str();
        confirm.trim();
        if (confirm == "OK") {
          transferOk = true;
          Serial.println("BLE: Confirmed OK");
          break;
        } else if (confirm == "FAIL") {
          Serial.println("BLE: Confirmed FAIL");
          break;
        }
        delay(200);
      }
    } else {
      recordsChar->writeValue("EMPTY");
      transferOk = true;
    }
  } else {
    recordsChar->writeValue("EMPTY");
    transferOk = true;
  }

  // Check for pending commands
  if (otaCtrlChar) {
    String cmd = otaCtrlChar->readValue().c_str();
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.println("BLE: Command: " + cmd);
      if (cmd == "DEBUG_ON") {
        cfg.debugMode = true;
        saveConfig(cfg);
      } else if (cmd == "DEBUG_OFF") {
        cfg.debugMode = false;
        saveConfig(cfg);
      } else if (cmd.startsWith("SLEEP:")) {
        int newSleep = cmd.substring(6).toInt();
        if (newSleep >= 60 && newSleep <= 3600) {
          cfg.sleepSeconds = newSleep;
          saveConfig(cfg);
        }
      }
    }
  }

  client->disconnect();
  delete client;

  Serial.println("BLE: Transfer done — success: " + String(transferOk));
  return transferOk;
}

// ===== CLEANUP =====
void bleServerStop() {
  if (bleServer) {
    BLEDevice::deinit(true);
    bleServer = nullptr;
  }
}