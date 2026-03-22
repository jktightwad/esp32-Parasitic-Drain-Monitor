#pragma once

// ===== BLE CLIENT (Collector side) =====
// Collector advertises as "VoltMon-Collector"
// VoltMon connects to it, sends records, receives confirmation

#include <NimBLEDevice.h>
#include "config.h"
#include "storage.h"

// ===== EXTERN — defined in collector_main.cpp =====
extern String cachedVoltMonVersion;
extern size_t cachedFirmwareSize;

// ===== COLLECTOR BLE STATE =====
static NimBLEServer*         collectorBleServer    = nullptr;
static NimBLECharacteristic* collectorRecordsChar  = nullptr;
static NimBLECharacteristic* collectorConfirmChar  = nullptr;
static NimBLECharacteristic* collectorDeviceIdChar = nullptr;
static NimBLECharacteristic* collectorOtaChar      = nullptr;
static NimBLECharacteristic* collectorOtaCtrlChar  = nullptr;

static bool   collectorConnected    = false;
static bool   collectorDataReceived = false;
static bool   otaStreamPending      = false;
static bool   otaReadyReceived      = false;
static String otaTargetDevice       = "";

static String receivedDeviceId      = "";
static String receivedRecords       = "";
static bool   receivedEmpty         = false;
static String pendingCommand        = "";

// ===== CHUNK REASSEMBLY =====
static String chunkBuffer    = "";
static int    expectedChunks = 0;
static int    receivedChunks = 0;

void resetChunkState() {
  chunkBuffer    = "";
  expectedChunks = 0;
  receivedChunks = 0;
}

// ===== SERVER CALLBACKS =====
class CollectorServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    collectorConnected    = true;
    collectorDataReceived = false;
    receivedDeviceId      = "";
    receivedRecords       = "";
    receivedEmpty         = false;
    otaStreamPending      = false;
    otaReadyReceived       = false;
    resetChunkState();
    // Clear OTA ctrl so VoltMon reads empty unless we set it
    Serial.println("BLE: VoltMon connected");
  }

  void onDisconnect(NimBLEServer* server) override {
    collectorConnected = false;
    Serial.println("BLE: VoltMon disconnected");
    NimBLEDevice::startAdvertising();
  }
};

// ===== DEVICE ID CALLBACKS =====
class DeviceIdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string val = pChar->getValue();
    String s = String(val.c_str());
    s.trim();

    // Check for OTA_READY signal from VoltMon — second dedicated OTA connection
    if (s == "OTA_READY") {
      Serial.println("BLE: VoltMon OTA_READY — starting stream immediately");
      otaReadyReceived = true;
      // Set otaStreamPending so loop() triggers the stream right away
      if (cachedFirmwareSize > 0) {
        otaStreamPending = true;
        otaTargetDevice  = receivedDeviceId.length() > 0 ? receivedDeviceId : "VoltMon";
      }
      return;
    }

    // Parse "deviceName:version" format
    int colonIdx = s.indexOf(':');
    if (colonIdx > 0) {
      receivedDeviceId      = s.substring(0, colonIdx);
      String reportedVersion = s.substring(colonIdx + 1);

      Serial.println("BLE: Device ID: " + receivedDeviceId +
                     " v" + reportedVersion);

      // Check if OTA update is available
      if (cachedVoltMonVersion.length() > 0 &&
          cachedFirmwareSize > 0 &&
          reportedVersion != cachedVoltMonVersion) {
        otaStreamPending = true;
        otaTargetDevice  = receivedDeviceId;
        Serial.println("BLE: OTA queued for " + receivedDeviceId +
                       " v" + reportedVersion + " -> " + cachedVoltMonVersion +
                       " (" + String(cachedFirmwareSize) + " bytes)");
      }
    } else {
      // Old format without version — just device name
      receivedDeviceId = s;
      Serial.println("BLE: Device ID: " + receivedDeviceId + " (no version)");
      collectorOtaCtrlChar->setValue("");
    }

    // Write any pending command
    if (pendingCommand.length() > 0) {
      collectorOtaCtrlChar->setValue(pendingCommand.c_str());
      Serial.println("BLE: Command queued: " + pendingCommand);
      pendingCommand = "";
    }
  }
};


// Helper — set confirm char, piggybacking OTA signal if pending
static void setConfirmValue(const String& base) {
  if (base == "OK" && otaStreamPending && cachedFirmwareSize > 0) {
    String otaConfirm = "OTA_START:" + String(cachedFirmwareSize);
    collectorConfirmChar->setValue(otaConfirm.c_str());
    collectorConfirmChar->notify();
    Serial.println("BLE: Confirm piggybacked with OTA signal: " + otaConfirm);
  } else {
    collectorConfirmChar->setValue(base.c_str());
    collectorConfirmChar->notify();
  }
}

// ===== RECORDS CALLBACKS =====
class RecordsCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) override {
    std::string raw = pChar->getValue();
    String val = String(raw.c_str());
    val.trim();

    if (val == "EMPTY") {
      Serial.println("BLE: VoltMon has no records");
      receivedEmpty         = true;
      collectorDataReceived = true;
      setConfirmValue("OK");
      return;
    }

    if (val == "TRANSFER_COMPLETE") {
      if (receivedChunks == expectedChunks && expectedChunks > 0) {
        Serial.println("BLE: Transfer complete — " + String(chunkBuffer.length()) + " bytes");
        int firstPipe = chunkBuffer.indexOf('|');
        if (firstPipe > 0) {
          receivedDeviceId = chunkBuffer.substring(0, firstPipe);
          receivedRecords  = chunkBuffer.substring(firstPipe + 1);
        } else {
          receivedRecords = chunkBuffer;
        }
        collectorDataReceived = true;
        setConfirmValue("OK");
      } else {
        Serial.println("BLE: Transfer incomplete — expected " +
                       String(expectedChunks) + " got " + String(receivedChunks));
        collectorConfirmChar->setValue("FAIL");
      }
      return;
    }

    if (val.startsWith("CHUNK:")) {
      int sep1 = val.indexOf(':', 6);
      int sep2 = val.indexOf(':', sep1 + 1);
      if (sep1 < 0 || sep2 < 0) return;

      int    seq   = val.substring(6, sep1).toInt();
      int    total = val.substring(sep1 + 1, sep2).toInt();
      String data  = val.substring(sep2 + 1);

      if (seq == 0) {
        chunkBuffer    = data;
        expectedChunks = total;
        receivedChunks = 1;
      } else {
        chunkBuffer   += data;
        receivedChunks++;
      }

      Serial.println("BLE: Chunk " + String(seq + 1) + "/" + String(total));
      return;
    }

    // Single-value transfer — parse deviceId|records
    int firstPipe = val.indexOf('|');
    if (firstPipe > 0) {
      receivedDeviceId = val.substring(0, firstPipe);
      receivedRecords  = val.substring(firstPipe + 1);
    } else {
      receivedRecords = val;
    }
    collectorDataReceived = true;
    setConfirmValue("OK");
    Serial.println("BLE: Records received — " + String(receivedRecords.length()) + " bytes");
  }
};

// ===== INIT COLLECTOR BLE =====
void collectorBleInit() {
  NimBLEDevice::init("VoltMon-Collector");
  NimBLEDevice::setMTU(512);

  collectorBleServer = NimBLEDevice::createServer();
  collectorBleServer->setCallbacks(new CollectorServerCallbacks());

  NimBLEService* service = collectorBleServer->createService(BLE_SERVICE_UUID);

  collectorDeviceIdChar = service->createCharacteristic(
    BLE_DEVICE_ID_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ
  );
  collectorDeviceIdChar->setCallbacks(new DeviceIdCallbacks());
  collectorDeviceIdChar->setValue("");

  collectorRecordsChar = service->createCharacteristic(
    BLE_RECORDS_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  collectorRecordsChar->setCallbacks(new RecordsCallbacks());
  collectorRecordsChar->setValue("");

  collectorConfirmChar = service->createCharacteristic(
    BLE_CONFIRM_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  collectorConfirmChar->setValue("");

  // OTA data — collector notifies VoltMon with firmware chunks
  collectorOtaChar = service->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );
  collectorOtaChar->setValue("");

  // OTA ctrl — collector writes commands/status VoltMon reads
  collectorOtaCtrlChar = service->createCharacteristic(
    BLE_OTA_CTRL_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  collectorOtaCtrlChar->setValue("");

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE: Collector advertising as VoltMon-Collector");
}

// ===== UPDATE ADVERTISING WITH OTA INFO VIA MANUFACTURER DATA =====
// Encodes firmware size as 4 bytes in manufacturer data — no characteristic read needed
void bleSetOtaAvailable() {
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->stop();

  NimBLEAdvertisementData advData;
  advData.setName("VoltMon-Collector");

  if (cachedVoltMonVersion.length() > 0 && cachedFirmwareSize > 0) {
    // Encode: company ID 0xFFFF (test) + 4-byte firmware size
    uint8_t mfData[6];
    mfData[0] = 0xFF;  // company ID low byte
    mfData[1] = 0xFF;  // company ID high byte
    uint32_t sz = (uint32_t)cachedFirmwareSize;
    memcpy(&mfData[2], &sz, 4);
    advData.setManufacturerData(std::string((char*)mfData, 6));
    Serial.println("BLE: OTA size in advert: " + String(cachedFirmwareSize));
  } else {
    Serial.println("BLE: OTA cleared from advert");
  }

  advertising->setAdvertisementData(advData);

  // Scan response carries service UUID so VoltMon can find service
  NimBLEAdvertisementData scanData;
  scanData.addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponseData(scanData);

  advertising->start();
}

void bleUpdateAdvertising() {
  bleSetOtaAvailable();
}

// ===== COMMAND / DATA ACCESSORS =====
void   bleSetPendingCommand(const String& cmd) { pendingCommand = cmd; }
bool   bleDataReceived()     { return collectorDataReceived; }
String bleGetDeviceId()      { return receivedDeviceId; }
String bleGetRecords()       { return receivedRecords; }
bool   bleGotEmpty()         { return receivedEmpty; }
bool   bleOtaStreamPending() { return otaStreamPending; }
void   bleOtaClearPending()  { otaStreamPending = false; otaTargetDevice = ""; }
bool   bleOtaReadyReceived() { return otaReadyReceived; }
void   bleOtaClearReady()    { otaReadyReceived = false; }
String bleOtaTargetDevice()  { return otaTargetDevice; }

void bleClearReceived() {
  collectorDataReceived = false;
  receivedDeviceId      = "";
  receivedRecords       = "";
  receivedEmpty         = false;
  resetChunkState();
}