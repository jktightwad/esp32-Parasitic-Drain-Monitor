#pragma once
#define TSLOG(x) do { \
  struct tm _ti; \
  if (getLocalTime(&_ti, 0)) { \
    char _tbuf[12]; \
    snprintf(_tbuf, sizeof(_tbuf), "%02d:%02d:%02d ", _ti.tm_hour, _ti.tm_min, _ti.tm_sec); \
    Serial.print(_tbuf); \
  } else { \
    Serial.print(String(millis()) + "ms "); \
  } \
  Serial.println(x); \
} while(0)


// ===== BLE CLIENT (Collector side) =====
// Collector advertises as "VoltMon-Collector"
// VoltMon connects to it, sends records, receives confirmation

#include <NimBLEDevice.h>
#include <time.h>
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
static volatile bool   otaAckReceived       = false;
static volatile bool   otaNextRequested     = false;
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
    TSLOG("BLE: VoltMon connected");
  }

  void onDisconnect(NimBLEServer* server) override {
    collectorConnected = false;
    TSLOG("BLE: VoltMon disconnected");
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
      TSLOG("BLE: VoltMon OTA_READY — starting stream immediately");
      otaReadyReceived = true;
      if (cachedFirmwareSize > 0) {
        otaStreamPending = true;
        otaTargetDevice  = receivedDeviceId.length() > 0 ? receivedDeviceId : "VoltMon";
      }
      return;
    }

    // Pull model: VoltMon requests next chunk
    if (s == "NEXT") {
      otaNextRequested = true;
      return;
    }

    // Parse "deviceName:version" format
    int colonIdx = s.indexOf(':');
    if (colonIdx > 0) {
      receivedDeviceId      = s.substring(0, colonIdx);
      String reportedVersion = s.substring(colonIdx + 1);

      TSLOG("BLE: Device ID: " + receivedDeviceId +
                     " v" + reportedVersion);

      // Save VoltMon's reported version so collector knows what it currently has
      extern String lastKnownVoltMonVer;
      lastKnownVoltMonVer = reportedVersion;

      // Check if OTA update is available — only if cached version is newer
      extern bool versionNewer(const String&, const String&);
      if (cachedVoltMonVersion.length() > 0 &&
          cachedFirmwareSize > 0 &&
          versionNewer(cachedVoltMonVersion, reportedVersion)) {
        otaStreamPending = true;
        otaTargetDevice  = receivedDeviceId;
        TSLOG("BLE: OTA queued for " + receivedDeviceId +
                       " v" + reportedVersion + " -> " + cachedVoltMonVersion +
                       " (" + String(cachedFirmwareSize) + " bytes)");
      }
    } else {
      // Old format without version — just device name
      receivedDeviceId = s;
      TSLOG("BLE: Device ID: " + receivedDeviceId + " (no version)");
      collectorOtaCtrlChar->setValue("");
    }

    // Write any pending command
    if (pendingCommand.length() > 0) {
      collectorOtaCtrlChar->setValue(pendingCommand.c_str());
      TSLOG("BLE: Command queued: " + pendingCommand);
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
    TSLOG("BLE: Confirm piggybacked with OTA signal: " + otaConfirm);
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
      TSLOG("BLE: VoltMon has no records");
      receivedEmpty         = true;
      collectorDataReceived = true;
      setConfirmValue("OK");
      return;
    }

    if (val == "TRANSFER_COMPLETE") {
      if (receivedChunks == expectedChunks && expectedChunks > 0) {
        TSLOG("BLE: Transfer complete — " + String(chunkBuffer.length()) + " bytes");
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
        TSLOG("BLE: Transfer incomplete — expected " +
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

      TSLOG("BLE: Chunk " + String(seq + 1) + "/" + String(total));
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
    TSLOG("BLE: Records received — " + String(receivedRecords.length()) + " bytes");
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

  // OTA data — collector indicates VoltMon with firmware chunks (indication = guaranteed delivery)
  collectorOtaChar = service->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::INDICATE
  );
  collectorOtaChar->setValue("");

  // OTA ctrl — collector writes commands/status VoltMon reads
  collectorOtaCtrlChar = service->createCharacteristic(
    BLE_OTA_CTRL_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  collectorOtaCtrlChar->setValue("");

  service->start();

  // Set up advertising params but don't start yet
  // bleSetOtaAvailable() will start advertising with manufacturer data intact
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);

  TSLOG("BLE: Collector BLE server ready");
}

// ===== UPDATE ADVERTISING WITH OTA INFO VIA MANUFACTURER DATA =====
void bleSetOtaAvailable() {
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

  if (cachedVoltMonVersion.length() > 0 && cachedFirmwareSize > 0) {
    uint8_t mfData[9];
    mfData[0] = 0xFF;
    mfData[1] = 0xFF;
    int maj = 0, min = 0, pat = 0;
    sscanf(cachedVoltMonVersion.c_str(), "%d.%d.%d", &maj, &min, &pat);
    mfData[2] = (uint8_t)maj;
    mfData[3] = (uint8_t)min;
    mfData[4] = (uint8_t)pat;
    uint32_t sz = (uint32_t)cachedFirmwareSize;
    memcpy(&mfData[5], &sz, 4);
    advertising->setManufacturerData(std::string((char*)mfData, 9));
    TSLOG("BLE: OTA in advert: v" + cachedVoltMonVersion +
                   " size=" + String(cachedFirmwareSize));
  } else {
    advertising->setManufacturerData("");
    TSLOG("BLE: OTA cleared from advert");
  }

  // Use NimBLEDevice::startAdvertising() — same method used after disconnects
  // Avoids the stop/start sequence that breaks discoverability
  NimBLEDevice::startAdvertising();
}

void bleUpdateAdvertising() {
  bleSetOtaAvailable();
}

// ===== OTA PUSH PENDING =====
// Set when VoltMon has switched to server mode and is advertising "VoltMon-OTA"
static bool otaPushPending   = false;
static size_t otaPushSize    = 0;

void bleSetOtaPushPending(size_t size) {
  otaPushPending = true;
  otaPushSize    = size;
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
bool   bleOtaAckReceived()   { return otaAckReceived; }
void   bleOtaClearAck()      { otaAckReceived = false; }
bool   bleOtaNextRequested() { return otaNextRequested; }
void   bleOtaClearNext()     { otaNextRequested = false; }
bool   bleOtaPushIsPending() { return otaPushPending; }
size_t bleOtaPushGetSize()   { return otaPushSize; }
void   bleOtaPushClear()     { otaPushPending = false; otaPushSize = 0; }
String bleOtaTargetDevice()  { return otaTargetDevice; }

void bleClearReceived() {
  collectorDataReceived = false;
  receivedDeviceId      = "";
  receivedRecords       = "";
  receivedEmpty         = false;
  resetChunkState();
}