#pragma once

#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <Update.h>
#include "config.h"
#include "storage.h"

// ===== STATE =====
static NimBLEServer*         bleServer       = nullptr;
static NimBLECharacteristic* bleRecordsChar  = nullptr;
static NimBLECharacteristic* bleConfirmChar  = nullptr;
static NimBLECharacteristic* bleDeviceIdChar = nullptr;
static NimBLECharacteristic* bleOtaChar      = nullptr;
static NimBLECharacteristic* bleOtaCtrlChar  = nullptr;

static bool   bleConnected       = false;
static bool   bleTransferDone    = false;
static bool   bleTransferSuccess = false;
static bool   bleOtaPending      = false;
static size_t otaExpectedSize    = 0;
static size_t otaReceivedBytes   = 0;
static bool   otaInProgress      = false;

// ===== SERVER CALLBACKS =====
class VoltMonServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    bleConnected       = true;
    bleTransferDone    = false;
    bleTransferSuccess = false;
    Serial.println("BLE: Collector connected");
  }
  void onDisconnect(NimBLEServer* server) override {
    bleConnected = false;
    Serial.println("BLE: Collector disconnected");
  }
};

// ===== CONFIRM CALLBACKS =====
class ConfirmCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    String s = String(val.c_str());
    s.trim();
    Serial.println("BLE confirm: " + s);
    if (s == "OK") {
      bleTransferSuccess = true;
      bleTransferDone    = true;
    } else if (s == "FAIL") {
      bleTransferSuccess = false;
      bleTransferDone    = true;
    }
  }
};

// ===== OTA CONTROL CALLBACKS =====
class OtaCtrlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    String s = String(val.c_str());
    s.trim();
    Serial.println("BLE OTA ctrl: " + s);

    if (s.startsWith("OTA_START:")) {
      otaExpectedSize  = s.substring(10).toInt();
      otaReceivedBytes = 0;
      otaInProgress    = true;
      if (!Update.begin(otaExpectedSize)) {
        Serial.println("OTA begin failed");
        otaInProgress = false;
      } else {
        Serial.println("OTA started — " + String(otaExpectedSize) + " bytes");
      }
    } else if (s == "OTA_END") {
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
    } else if (s == "OTA_ABORT") {
      Update.abort();
      otaInProgress    = false;
      otaReceivedBytes = 0;
    }
  }
};

// ===== OTA DATA CALLBACKS =====
class OtaDataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
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

// ===== INIT BLE SERVER =====
void bleServerInit(const DeviceConfig& cfg) {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(512);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new VoltMonServerCallbacks());

  NimBLEService* service = bleServer->createService(BLE_SERVICE_UUID);

  bleDeviceIdChar = service->createCharacteristic(
    BLE_DEVICE_ID_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  bleDeviceIdChar->setValue(cfg.deviceName);

  bleRecordsChar = service->createCharacteristic(
    BLE_RECORDS_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  bleRecordsChar->setValue("");

  bleConfirmChar = service->createCharacteristic(
    BLE_CONFIRM_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  bleConfirmChar->setCallbacks(new ConfirmCallbacks());

  bleOtaChar = service->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  bleOtaChar->setCallbacks(new OtaDataCallbacks());

  bleOtaCtrlChar = service->createCharacteristic(
    BLE_OTA_CTRL_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  bleOtaCtrlChar->setCallbacks(new OtaCtrlCallbacks());

  service->start();
  Serial.println("BLE server started — " + String(cfg.deviceName));
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

  if (!recordsChar || !confirmChar || !deviceIdChar) {
    Serial.println("BLE: Required characteristics not found");
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  // Send device ID
  deviceIdChar->writeValue(cfg.deviceName, true);

  bool transferOk = false;

  if (hasRecords) {
    String records = loadPendingRecords();
if (records.length() > 0) {
      String payload = String(cfg.deviceName) + "|" + records;
      recordsChar->writeValue(payload.c_str(), true);
      Serial.println("BLE: Sent " + String(records.length()) + " bytes");

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

  // Check for pending commands
  if (otaCtrlChar) {
    std::string cmd = otaCtrlChar->readValue();
    String cs = String(cmd.c_str());
    cs.trim();
    if (cs.length() > 0) {
      Serial.println("BLE: Command: " + cs);
      if (cs == "DEBUG_ON") {
        cfg.debugMode = true;
        saveConfig(cfg);
      } else if (cs == "DEBUG_OFF") {
        cfg.debugMode = false;
        saveConfig(cfg);
      } else if (cs.startsWith("SLEEP:")) {
        int newSleep = cs.substring(6).toInt();
        if (newSleep >= 60 && newSleep <= 3600) {
          cfg.sleepSeconds = newSleep;
          saveConfig(cfg);
        }
      }
    }
  }

  client->disconnect();
  NimBLEDevice::deleteClient(client);

  Serial.println("BLE: Transfer done — success: " + String(transferOk));
  return transferOk;
}

// ===== CLEANUP =====
void bleServerStop() {
  if (bleServer) {
    NimBLEDevice::deinit(true);
    bleServer = nullptr;
  }
}