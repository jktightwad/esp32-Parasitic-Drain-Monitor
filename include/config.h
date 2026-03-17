#pragma once

#include "secrets.h"

// ==== DEBUG MODE ====
#define DEBUG_MODE  true  // set true to enable remote debugging
#define DEBUG_SLEEP_SECONDS  15      // loop interval in debug mode

// ===== NETWORK =====
#define AIO_SERVER      "io.adafruit.com"
#define AIO_PORT        1883
#define NTP_SERVER      "pool.ntp.org"
#define WIFI_ATTEMPTS_PER_NETWORK  3   // attempts per network before moving to next

// ===== FIRMWARE =====
#define FIRMWARE_VERSION    "1.0.36" //added debug

// ===== OTA ===== //additing so it triggers
#define OTA_VERSION_URL  "https://raw.githubusercontent.com/jktightwad/esp32-Parasitic-Drain-Monitor/main/firmware/version.txt"
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/jktightwad/esp32-Parasitic-Drain-Monitor/main/firmware/firmware.bin"

// ===== PINS =====
#define PIN_TRUCK_SOURCE     0
//#define PIN_TRUCK_ADC        1
//#define PIN_BATT_ADC         4
#define PIN_TRUCK_ADC        4
#define PIN_BATT_ADC         1
#define PIN_CHARGE_MOSFET    7
#define PIN_DS3231_SDA       9
#define PIN_DS3231_SCL       8

// ===== VOLTAGE DIVIDER RATIOS =====
#define TRUCK_DIVIDER_RATIO  (55.0 / 9.9)
#define BATT_DIVIDER_RATIO   (14.7 / 10.0)

// ===== ADC =====
#define ADC_SAMPLES          16

// ===== FILE SETTINGS =====
#define PENDING_FILE         "/pending.csv"
#define ARCHIVE_FILE         "/archive.csv"
#define DEBUG_FILE           "/debug.log"
#define STATE_FILE           "/state.json"
#define MAX_ARCHIVE_SIZE     800000
#define MAX_DEBUG_SIZE       100000
#define MAX_PENDING_RECORDS  100

// ===== TIMING =====
#define SLEEP_SECONDS        300    // testing — change to 300 for final
#define UPLOAD_EVERY         6     // testing — change to 12 for final
const int CHARGE_CHECK_SECONDS = (SLEEP_SECONDS < 60) ? SLEEP_SECONDS : 60;

// ===== VOLTAGE THRESHOLDS =====
#define VOLTAGE_RUNNING      12.9
#define VOLTAGE_LOW          11.5
#define BATT_LOW             2.80
#define BATT_START_CHARGE    3.50
#define BATT_STOP_CHARGE     3.58

// ===== TRUCK VOLTAGE CALIBRATION =====
// Format: {raw_reading, actual_voltage}
const float TRUCK_CAL_TABLE[][2] = {
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
const int TRUCK_CAL_TABLE_SIZE = 12;

// ===== BATTERY VOLTAGE CALIBRATION =====
const float BATT_CAL_TABLE[][2] = {
  {3.469, 3.495},
  {4.140, 4.110}
};
const int BATT_CAL_TABLE_SIZE = 2;