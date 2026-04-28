#ifndef CONFIG_H
#define CONFIG_H

// ===== FIRMWARE VERSION =====
// Bit 15 = 0 for robot/base (not repeater)
#define FIRMWARE_VERSION 0x0002

// ----- LED configuration -----
#define LED_DATA_PIN 25
#define LED_NUM_LEDS 2

// ----- Button configuration -----
#define BUTTON_BLUE_PIN 27
#define BUTTON_RED_PIN 26
#define BUTTON_DEBOUNCE_MS 30

// ----- Base capture configuration -----
#define CAPTURE_TIME_MS 3000      // Time to hold button to capture the base
#define CAPTURE_BLINK_PERIOD_MS 400  // LED blink period during capture

// ----- Base colors -----
#define BASE_COLOR_NEUTRAL 0xFF
#define BASE_COLOR_BLUE    0x01
#define BASE_COLOR_RED     0x02

// ----- BLE configuration -----
#define MAX_BLE_NAME_LEN 16
#define BLE_NAME_PREFIX "RBM_"

// ----- BLE Advertising configuration -----
#define ADV_MANUFACTURER_ID     0xFFFF
#define ADV_INTERVAL            0x0320  // 500ms (0x0320 * 0.625ms)
// Layout: type(1) + seq(2, LE) + color(1) + change_count(1) + score_seconds(2, LE) + fw_ver(2, LE)
// seq is placed near the front so it falls inside Android BLE controllers' ADV de-duplication
// hash window (typically the first ~8 bytes), which prevents stale-cache delivery on idle bases.
#define ADV_BASE_STATUS_SIZE    9
#define ADV_UPDATE_INTERVAL_MS  200
#define DATA_TYPE_BASE_ADV      0x04

// ----- BLE GATT configuration -----
#define BASE_BLE_TIMEOUT_MS     10000
#define BLE_CMD_KEEPALIVE       0x00
#define GATT_BASE_STATUS_MSG_SIZE   7   // type(1) + color(1) + change_count(1) + score_seconds(2) + fw_ver(2)
#define GATT_BASE_STATUS_INTERVAL_MS 1000

// ----- Serial protocol configuration -----
#define SERIAL_START_BYTE       0xAA
#define SERIAL_CMD_GET_INFO     0x01
#define SERIAL_CMD_SET_NAME     0x02
#define ROBOT_NAME_MIN_LEN      3
#define ROBOT_NAME_MAX_LEN      10

#endif // CONFIG_H
