#pragma once

#include "secrets.h"

// ===== VOLTMON VERSION =====
#define VOLTMON_VERSION     "2.1.3"

// ===== COLLECTOR VERSION =====
#define COLLECTOR_VERSION   "1.1.3"

// ===== VOLTMON OTA =====
#define VOLTMON_OTA_VERSION_URL  "https://raw.githubusercontent.com/jktightwad/esp32-Parasitic-Drain-Monitor/main/firmware/voltmon/version.txt"
#define VOLTMON_OTA_FIRMWARE_URL "https://raw.githubusercontent.com/jktightwad/esp32-Parasitic-Drain-Monitor/main/firmware/voltmon/firmware.bin"

// ===== COLLECTOR OTA =====
#define COLLECTOR_OTA_VERSION_URL  "https://raw.githubusercontent.com/jktightwad/esp32-Parasitic-Drain-Monitor/main/firmware/collector/version.txt"
#define COLLECTOR_OTA_FIRMWARE_URL "https://raw.githubusercontent.com/jktightwad/esp32-Parasitic-Drain-Monitor/main/firmware/collector/firmware.bin"

// ===== BLE =====
#define BLE_DEVICE_NAME          "VoltMon"
#define BLE_SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c3319123"
#define BLE_RECORDS_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CONFIRM_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_DEVICE_ID_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define BLE_OTA_CHAR_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26ab"
#define BLE_OTA_CTRL_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26ac"

// ===== NETWORK =====
#define AIO_SERVER      "io.adafruit.com"
#define AIO_PORT        1883
#define NTP_SERVER      "pool.ntp.org"

// ===== WIFI =====
#define WIFI_ATTEMPTS_PER_NETWORK  1   // fallback only — single attempt per network

// ===== PINS — VoltMon =====
#define PIN_TRUCK_SOURCE     0
#define PIN_TRUCK_ADC        4
#define PIN_BATT_ADC         1
#define PIN_CHARGE_MOSFET    7
#define PIN_DS3231_SDA       9
#define PIN_DS3231_SCL       8

// ===== PINS — Collector =====
#define VM_PIN_NEOPIXEL      3    // renamed to avoid collision with framework PIN_NEOPIXEL
#define VM_NEOPIXEL_COUNT    18
#define VM_NEOPIXEL_BRIGHTNESS  80

// ===== LED LAYOUT =====
#define LED_WIFI_START       0
#define LED_WIFI_COUNT       3
#define LED_ACTIVITY_START   3
#define LED_ACTIVITY_COUNT   12
#define LED_UPLOAD_START     15
#define LED_UPLOAD_COUNT     3

// ===== VOLTAGE DIVIDER RATIOS =====
#define TRUCK_DIVIDER_RATIO  (55.0 / 9.9)
#define BATT_DIVIDER_RATIO   (14.7 / 10.0)

// ===== ADC =====
#define ADC_SAMPLES          16

// ===== FILE SETTINGS =====
#define MAX_ARCHIVE_SIZE     800000
#define MAX_DEBUG_SIZE       100000
#define MAX_PENDING_RECORDS  100

// ===== TIMING — VoltMon =====
#define SLEEP_SECONDS        30 //how many seconds to sleep
#define UPLOAD_EVERY         3 //how many to store before posting
static const int CHARGE_CHECK_SECONDS = (SLEEP_SECONDS < 60) ? SLEEP_SECONDS : 60;

// ===== TIMING — Collector =====
#define COLLECTOR_BLE_SCAN_SECONDS   10
#define COLLECTOR_UPLOAD_RETRY_MS    30000

// ===== VOLTAGE THRESHOLDS =====
#define VOLTAGE_RUNNING      12.9
#define VOLTAGE_LOW          11.5
#define BATT_LOW             2.80
#define BATT_START_CHARGE    3.50
#define BATT_STOP_CHARGE     3.58

// ===== FALLBACK DEVICE NAME =====
#define FALLBACK_DEVICE_NAME  "2021AT4"

// ===== TRUCK VOLTAGE CALIBRATION =====
static const float TRUCK_CAL_TABLE[][2] = {
  {4.006,  4.0},
  {4.983,  5.0},
  {5.969,  6.0},
  {7.150,  7.0},
  {8.172,  8.0},
  {9.211,  9.0},
  {10.239, 10.0},
  {11.250, 11.0},
  {12.244, 12.0},
  {13.239, 13.0},
  {14.289, 14.0},
  {15.417, 15.0}
};
static const int TRUCK_CAL_TABLE_SIZE = 12;

// ===== BATTERY VOLTAGE CALIBRATION =====
static const float BATT_CAL_TABLE[][2] = {
  {3.469, 3.495},
  {4.140, 4.110}
};
static const int BATT_CAL_TABLE_SIZE = 2;

// ===== DEBUG MODE =====
#define DEBUG_MODE           false
#define DEBUG_SLEEP_SECONDS  15