#pragma once

// ===== BLE CLIENT (Collector side) =====
// Collector advertises as "VoltMon-Collector"
// VoltMon connects to it, sends records, receives confirmation

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "config.h"
#include "storage.h"

// ===== COLLECTOR BLE STATE =====
static BLEServer*         collectorBleServer     = nullptr;
static BLECharacteristic* collectorRecordsChar   = nullptr;
static BLECharacteristic* collectorConfirmChar   = nullptr;
static BLECharacteristic* collectorDeviceIdChar  = nullptr;
static BLECharacteristic* collectorOtaChar       = nullptr;
static BLECharacteristic* collectorOtaCtrlChar   = nullptr;

static bool collectorConnected      = false;
static bool collectorDataReceived   = false;
static bool collectorOtaRequested   = false;

// Buffer for received records before upload
static String receivedDeviceId      = "";
static String receivedRecords       = "";
static bool   receivedEmpty         = false;

// Pending command to send to VoltMon
static String pendingCommand        = "";

// ===== CHUNK REASSEMBLY =====
static String  chunkBuffer          = "";
static int     expectedChunks       = 0;
static int     receivedChunks       = 0;
static bool    transferComplete     = false;

void resetChunkState() {
  chunkBuffer     = "";
  expectedChunks  = 0;
  receivedChunks  = 0;
  transferComplete = false;
}

// ===== SERVER CALLBACKS =====
class CollectorServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    collectorConnected    = true;
    collectorDataReceived = false;
    receivedDeviceId      = "";
    receivedRecords       = "";
    receivedEmpty         = false;
    resetChunkState();
    Serial.println("BLE: VoltMon connected");
  }

  void onDisconnect(BLEServer* server) override {
    collectorConnected = false;
    Serial.println("BLE: VoltMon disconnected");
    // Restart advertising
    collectorBleServer->startAdvertising();
  }
};

// ===== DEVICE ID CHARACTERISTIC CALLBACKS =====
class DeviceIdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    receivedDeviceId = pChar->getValue().c_str();
    receivedDeviceId.trim();
    Serial.println("BLE: Device ID received: " + receivedDeviceId);

    // Write any pending command back so VoltMon can read it
    if (pendingCommand.length() > 0) {
      collectorOtaCtrlChar->setValue(pendingCommand.c_str());
      Serial.println("BLE: Queued command sent: " + pendingCommand);
      pendingCommand = "";
    } else {
      collectorOtaCtrlChar->setValue("");
    }
  }
};

// ===== RECORDS CHARACTERISTIC CALLBACKS =====
class RecordsCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String val = pChar->getValue().c_str();
    val.trim();

    if (val == "EMPTY") {
      Serial.println("BLE: VoltMon has no pending records");
      receivedEmpty         = true;
      collectorDataReceived = true;
      // Confirm immediately
      collectorConfirmChar->setValue("OK");
      return;
    }

    if (val == "TRANSFER_COMPLETE") {
      if (receivedChunks == expectedChunks && expectedChunks > 0) {
        Serial.println("BLE: Transfer complete — " + String(chunkBuffer.length()) + " bytes");
        // Parse device ID and records from buffer
        // Format: deviceName|record1|record2|...
        int firstPipe = chunkBuffer.indexOf('|');
        if (firstPipe > 0) {
          receivedDeviceId = chunkBuffer.substring(0, firstPipe);
          receivedRecords  = chunkBuffer.substring(firstPipe + 1);
        } else {
          receivedRecords = chunkBuffer;
        }
        collectorDataReceived = true;
        // Confirm receipt — upload happens after disconnect
        collectorConfirmChar->setValue("OK");
      } else {
        Serial.println("BLE: Transfer incomplete — expected " +
                       String(expectedChunks) + " got " + String(receivedChunks));
        collectorConfirmChar->setValue("FAIL");
      }
      return;
    }

    // Handle chunked data: CHUNK:<seq>:<total>:<data>
    if (val.startsWith("CHUNK:")) {
      int sep1 = val.indexOf(':', 6);
      int sep2 = val.indexOf(':', sep1 + 1);
      if (sep1 < 0 || sep2 < 0) return;

      int seq   = val.substring(6, sep1).toInt();
      int total = val.substring(sep1 + 1, sep2).toInt();
      String data = val.substring(sep2 + 1);

      if (seq == 0) {
        // First chunk — initialize
        chunkBuffer    = data;
        expectedChunks = total;
        receivedChunks = 1;
      } else {
        chunkBuffer   += data;
        receivedChunks++;
      }

      Serial.println("BLE: Chunk " + String(seq + 1) + "/" + String(total) +
                     " (" + String(data.length()) + " bytes)");
      return;
    }

    // Simple single-value transfer (no chunking needed)
    // Parse device ID and records
    int firstPipe = val.indexOf('|');
    if (firstPipe > 0) {
      receivedDeviceId = val.substring(0, firstPipe);
      receivedRecords  = val.substring(firstPipe + 1);
    } else {
      receivedRecords = val;
    }
    collectorDataReceived = true;
    collectorConfirmChar->setValue("OK");
    Serial.println("BLE: Records received — " + String(receivedRecords.length()) + " bytes");
  }
};

// ===== INIT COLLECTOR BLE =====
void collectorBleInit() {
  BLEDevice::init("VoltMon-Collector");
  BLEDevice::setMTU(512);

  collectorBleServer = BLEDevice::createServer();
  collectorBleServer->setCallbacks(new CollectorServerCallbacks());

  BLEService* service = collectorBleServer->createService(BLE_SERVICE_UUID);

  // Device ID characteristic — VoltMon writes its name here
  collectorDeviceIdChar = service->createCharacteristic(
    BLE_DEVICE_ID_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );
  collectorDeviceIdChar->setCallbacks(new DeviceIdCallbacks());
  collectorDeviceIdChar->setValue("");

  // Records characteristic — VoltMon writes pending records here
  collectorRecordsChar = service->createCharacteristic(
    BLE_RECORDS_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  collectorRecordsChar->addDescriptor(new BLE2902());
  collectorRecordsChar->setCallbacks(new RecordsCallbacks());
  collectorRecordsChar->setValue("");

  // Confirm characteristic — Collector writes OK/FAIL here
  collectorConfirmChar = service->createCharacteristic(
    BLE_CONFIRM_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  collectorConfirmChar->addDescriptor(new BLE2902());
  collectorConfirmChar->setValue("");

  // OTA data characteristic — Collector writes firmware chunks here
  collectorOtaChar = service->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  collectorOtaChar->addDescriptor(new BLE2902());
  collectorOtaChar->setValue("");

  // OTA control characteristic — Collector writes commands here
  // VoltMon reads this after writing device ID
  collectorOtaCtrlChar = service->createCharacteristic(
    BLE_OTA_CTRL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
  );
  collectorOtaCtrlChar->setValue("");

  service->start();

  // Start advertising
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE: Collector advertising as VoltMon-Collector");
}

// ===== SET PENDING COMMAND =====
// Called when MQTT control message received
// Command will be sent to VoltMon on next BLE connection
void bleSetPendingCommand(const String& cmd) {
  pendingCommand = cmd;
  Serial.println("BLE: Command queued for VoltMon: " + cmd);
}

// ===== CHECK IF DATA RECEIVED =====
bool bleDataReceived() {
  return collectorDataReceived;
}

// ===== GET RECEIVED DATA =====
String bleGetDeviceId()  { return receivedDeviceId; }
String bleGetRecords()   { return receivedRecords; }
bool   bleGotEmpty()     { return receivedEmpty; }

// ===== CLEAR RECEIVED DATA =====
void bleClearReceived() {
  collectorDataReceived = false;
  receivedDeviceId      = "";
  receivedRecords       = "";
  receivedEmpty         = false;
  resetChunkState();
}

// ===== SEND OTA TO VOLTMON =====
// Called when OTA firmware is ready to send
// firmware: pointer to firmware data
// size: firmware size in bytes
bool bleSendOTA(const uint8_t* firmware, size_t size) {
  if (!collectorConnected) {
    Serial.println("BLE OTA: VoltMon not connected");
    return false;
  }

  Serial.println("BLE OTA: Starting — " + String(size) + " bytes");

  // Send OTA start command
  String startCmd = "OTA_START:" + String(size);
  collectorOtaCtrlChar->setValue(startCmd.c_str());
  collectorOtaCtrlChar->notify();
  delay(500);

  // Send firmware in chunks
  const size_t CHUNK_SIZE = 400;
  size_t sent = 0;

  while (sent < size) {
    if (!collectorConnected) {
      Serial.println("BLE OTA: Connection lost at " + String(sent) + "/" + String(size));
      return false;
    }

    size_t chunkLen = min(CHUNK_SIZE, size - sent);
    collectorOtaChar->setValue((uint8_t*)(firmware + sent), chunkLen);
    collectorOtaChar->notify();
    sent += chunkLen;

    Serial.println("BLE OTA: " + String(sent) + "/" + String(size));
    delay(50);
  }

  // Send OTA end
  collectorOtaCtrlChar->setValue("OTA_END");
  collectorOtaCtrlChar->notify();
  delay(500);

  Serial.println("BLE OTA: Complete");
  return true;
}