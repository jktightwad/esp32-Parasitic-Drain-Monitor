#include "config.h"
#include "storage.h"
#include <NimBLEDevice.h>
#include "ble_server.h"
#include "esp_task_wdt.h"

bool executeBLETransfer(DeviceConfig& cfg, bool hasRecords) {
  esp_task_wdt_deinit();
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setMTU(512);
  esp_task_wdt_init(120, false);
  bool ok = bleScanAndTransfer(cfg, hasRecords);
  NimBLEDevice::deinit(true);
  return ok;
}