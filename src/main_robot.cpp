#include "main_robot.h"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Bluepad32.h>
#include <uni.h>
#include <esp_system.h>

// New standardized modules
#include "logger.h"
#include "shared_data.h"
#include "shared_i2c.h"

// Balance control modules (runs on Core 0 - isolated task)
#include "imu.h"
#include "motor_hardware.h"
#include "balance_controller.h"

// Communication modules (run on Core 1 - Arduino loop())
#include "cc1101.h"
#include "ble_server.h"
#include "controller_handler.h"
#include "controller_mapping.h"
#include "central_controller.h"
#include "led_controller.h"
#include "adc.h"
#include "atecc_crypto.h"
#include "calibration_manager.h"
#include "scan.h"
#include "serial_protocol.h"
#include "subprogram.h"
#include "melody_player.h"
#include "temperature_sensor.h"
#include "config.h"

// Use the global Bluepad32 instance from the library
extern Bluepad32 BP32;

// Task handles
static TaskHandle_t balanceTaskHandle = NULL;
static TaskHandle_t cc1101TaskHandle = nullptr;

// CC1101 dedicated task (runs on Core 0, independent of BLE/serial on Core 1)
static void cc1101Task(void* parameter) {
    for (;;) {
        MyCC1101::update();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Statistics tracking
static unsigned long lastStatsLogMs = 0;

static void printStats() {
    unsigned long now = millis();
    if (now - lastStatsLogMs >= STATS_INTERVAL_MS) {
        lastStatsLogMs = now;
        
        MyCC1101::Stats stats = MyCC1101::getAndResetStats();
        
        Logger::infof("CC1101", "Stats [%ums]: RX=%u (ping=%u, upd=%u, scan=%u), TX=%u (ping=%u, upd=%u, scan=%u)",
                      STATS_INTERVAL_MS, 
                      stats.totalRx(),
                      stats.rx_ping, stats.rx_update_status, stats.rx_scan,
                      stats.totalTx(),
                      stats.tx_ping, stats.tx_update_status, stats.tx_scan);
    }
}

void robot_setup() {
    unsigned long bootStartTime = millis();

    // === Early initialization (show we're alive) ===
    LEDController::init();
    LEDController::setEyes(CRGB::White);  // White = loading
    LEDController::showLEDs();
    
    // === Logger initialization ===
    Logger::init(LogLevel::INFO);
    Logger::info("MAIN", "=== Robomate Initialization ===");
    
    // Print Bluetooth MAC address for identification
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    Logger::infof("MAIN", "BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // === Hardware setup ===
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // === Shared system initialization ===
    Logger::info("MAIN", "Initializing shared systems...");
    SharedData::init();
    
    if (!SharedI2C::init()) {
        Logger::critical("MAIN", "Shared I2C initialization failed!");
        LEDController::setEyes(CRGB::Red);
        LEDController::showLEDs();
        while (true) { delay(1000); }  // Halt on critical error
    }
    
    // === Crypto chip initialization ===
    ATECCCrypto& crypto = ATECCCrypto::getInstance();
    if (!crypto.init()) {
        Logger::error("MAIN", "ATECC508A initialization failed!");
        LEDController::setEyes(CRGB::Red);
        LEDController::showLEDs();
        while (true) { delay(1000); }  // Halt on critical error
    }
    
    // === Motor & Balance system initialization ===
    Logger::info("MAIN", "Initializing motor hardware...");
    Wire.begin(19, 23);  // Primary I2C for IMU (100kHz)
    Wire.setClock(PRIMARY_I2C_CLOCK);
    Logger::info("MAIN", "Scanning for I2C devices on default (Wire) bus...");
    uint8_t devicesFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Logger::infof("MAIN", "I2C Device found: 0x%02X", addr);
            devicesFound++;
        }
    }
    if (devicesFound == 0) {
        Logger::info("MAIN", "No I2C devices found on default bus.");
    }
    
    MotorHardware::init();
    MotorHardware::initializeMotors();
    BalanceController::init();
    
    // === Temperature sensor initialization ===
    Logger::info("MAIN", "Initializing temperature sensors...");
    if (!TemperatureSensor::init()) {  // Uses both Wire and SharedI2C
        Logger::critical("MAIN", "Temperature sensor initialization failed - not all sensors found!");
        LEDController::setLED(LED_L_BOT, CRGB::Purple);
        LEDController::setLED(LED_R_BOT, CRGB::Purple);
        LEDController::showLEDs();
        while (true) { delay(1000); }  // Halt on critical error
    }
    
    // === Communication modules initialization ===
    Logger::info("MAIN", "Initializing communication modules...");
    SerialProtocol::init();  // Must be before BLEServer (provides robot name)
    ControllerMapping::init();  // Controller input mapping
    ControllerHandler::init();
    CentralController::init();  // Central controller (priority: subprogram > BLE > physical)
    BLEServer::init();
    MyCC1101::init();

    // === Subprogram module initialization ===
    Subprogram::init();
    
    // === Melody storage initialization ===
    MelodyPlayer::init();
    
    // === Configure Bluetooth allowlist ===
    uni_bt_allowlist_remove_all();
    uni_bt_allowlist_set_enabled(false);
    
    // === Initialize system state ===
    SharedData::SystemState systemState = {};
    systemState.batteryMillivolts = ADC::batteryMillivolts();
    systemState.centralConnected = false;
    systemState.speedCoefficient = 1.0f;
    systemState.torqueCoefficient = 1.0f;
    SharedData::setSystemState(systemState);
    
    // === Battery level check before starting balance ===
    // Do initial battery read (average of 16 samples)
    long batterySum = 0;
    for (int i = 0; i < 16; i++) {
        batterySum += analogReadMilliVolts(BATTERY_ADC_PIN);
    }
    uint32_t batteryMV = batterySum * VOLTAGE_DIVIDER_RATIO / 16;
    
    if (batteryMV < BATTERY_MIN_MILLIVOLTS) {
        Logger::errorf("MAIN", "Battery too low: %u mV (min: %u mV)", batteryMV, BATTERY_MIN_MILLIVOLTS);
        LEDController::setLED(LED_L_BOT, CRGB::Red);
        LEDController::setLED(LED_R_BOT, CRGB::Red);
        LEDController::showLEDs();
        while (true) { delay(1000); }  // Halt - battery too low
    } else {
        Logger::infof("MAIN", "Battery level OK: %u mV", batteryMV);
    }
    
    // === Start dual-core operation ===
    Logger::info("MAIN", "=== Starting Dual-Core Operation ===");
    
    LEDController::setEyes(CRGB::Blue);  // Blue = ready
    LEDController::showLEDs();

    Logger::infof("MAIN", "Setup finished, stack: %u", uxTaskGetStackHighWaterMark(NULL));
    Logger::infof("MAIN", "Boot time: %lu ms", millis() - bootStartTime);

    BaseType_t res = xTaskCreatePinnedToCore(
        BalanceController::balanceTask,  // Task function
        "BalanceControl",             // Task name
        16384,                         // Stack: 16KB
        NULL,                         // Parameters
        5,                            // Priority (highest user task — must not be starved)
        &balanceTaskHandle,           // Task handle
        1                             // Core 1 (shared with Arduino loop at priority 1)
    );
    if (res != pdPASS) {
        Logger::critical("MAIN", "Failed to create Balance control task on Core 1");
        LEDController::setEyes(CRGB::Red);
        LEDController::showLEDs();
        while (true) { delay(1000); }  // Halt on critical error
    }
    Logger::info("MAIN", "✓ Balance control task started on Core 1 (priority 5)");
    heap_caps_check_integrity_all(true);
    
    // === Start CC1101 on dedicated Core 0 task ===
    // When CC1101 is disabled, this task only handles BLE ping timing
    // When enabled, it handles full RF receive/transmit independent of BLE/serial
    BaseType_t res2 = xTaskCreatePinnedToCore(
        cc1101Task,        // Task function
        "CC1101Task",      // Name
        DISABLE_CC1101 ? 4096 : 8192, // 4KB when disabled (BLE pings only), 8KB when enabled (packet buffers)
        NULL,              // Parameters
        2,                 // Priority (lower than balance at 5, higher than Arduino loop at 1)
        &cc1101TaskHandle, // Handle
        0                  // Core 0 (Arduino loop + BLE run on Core 1)
    );
    if (res2 != pdPASS) {
        Logger::critical("MAIN", "Failed to create CC1101 task on Core 0");
        LEDController::setEyes(CRGB::Red);
        LEDController::showLEDs();
        while (true) { delay(1000); }  // Halt on critical error
    }
    Logger::infof("MAIN", "✓ CC1101 task started on Core 0%s", DISABLE_CC1101 ? " (sub-GHz disabled, BLE pings only)" : "");
    
    Logger::info("MAIN", "✓ Communications will run in loop() on Core 1");

    if (CRYPTO_DEBUG_MODE) {
        crypto.demo();
    }
}

void robot_loop() {
    // === Update communication modules ===
    SerialProtocol::update();  // Handle serial protocol commands
    ControllerHandler::update();
    BLEServer::update();
    // MyCC1101::update() runs on dedicated Core 0 task (cc1101Task)
    ADC::update();
    TemperatureSensor::update();  // Update temperature sensor readings
    
    // === Handle controller-specific logic using mapped robot inputs ===
    // Uses CentralController for priority: subprogram > BLE controller > physical controller
    SharedData::RobotInputs robotInputs = CentralController::getRobotInputs();
    
    if (robotInputs.isConnected) {
        // Handle subprogram execution (buttons 1-5)
        Subprogram::handleButton(1, robotInputs.subprogram1);
        Subprogram::handleButton(2, robotInputs.subprogram2);
        Subprogram::handleButton(3, robotInputs.subprogram3);
        Subprogram::handleButton(4, robotInputs.subprogram4);
        Subprogram::handleButton(5, robotInputs.subprogram5);
        
        // Eye color cycling when subprogram is not running
        // Uses updateEyeColorCycling which handles edge detection internally
        if (!Subprogram::isRunning()) {
            LEDController::updateEyeColorCycling(robotInputs.prevEyeColor, robotInputs.nextEyeColor);
        }

        MyCC1101::setControllerBattery(ControllerHandler::getControllerBattery());
        MyCC1101::setControllerModel(ControllerHandler::getControllerModel());
        
        // Scan handling when subprogram is not running
        // Note: Scan is disabled in crawl mode (when robot has fallen)
        if (!Subprogram::isRunning()) {
            bool shouldRumble = false;
            SharedData::MotorState motorStateForScan = SharedData::getMotorState();
            // Pass scan action directly - both buttons map to same action
            Scan::update(robotInputs.scan, robotInputs.scan, shouldRumble, motorStateForScan.hasFallen);
            if (shouldRumble) {
                ControllerHandler::Rumble();
            }
        }
    } else {
        MyCC1101::setControllerBattery(0);
        MyCC1101::setControllerModel(0);
    }
    
    // === Update subprogram execution ===
    Subprogram::update();
    
    // === Handle fall detection ===
    SharedData::MotorState motorState = SharedData::getMotorState();
    MyCC1101::setIsFallDetected(motorState.hasFallen);
    
    // Note: motorState.hasFallen is used above for scan crawl mode check
    
    // === Apply pending LED updates from Core 0 (RF path) ===
    MyCC1101::applyPendingLedUpdate();
    
    // === Update LED animations ===
    LEDController::update();
    
    // === Print CC1101 statistics periodically ===
    printStats();
    
    delay(10);
}

