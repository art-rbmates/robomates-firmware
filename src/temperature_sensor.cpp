#include "temperature_sensor.h"
#include "shared_i2c.h"
#include "config.h"
#include "logger.h"
#include <Wire.h>
#include <Adafruit_TMP117.h>

static const char* MODULE = "TempSensor";

// TMP117 I2C addresses
static const uint8_t TMP117_ADDR_MAIN = 0x49;   // Main board sensor
static const uint8_t TMP117_ADDR_RIGHT = 0x48;  // Right board sensor
static const uint8_t TMP117_ADDR_LEFT = 0x48;   // Left board sensor (on SharedI2C)

// Temperature read interval (ms)
static const unsigned long TEMP_READ_INTERVAL_MS = 1000;

// Sensor instances
static Adafruit_TMP117* s_mainSensor = nullptr;
static Adafruit_TMP117* s_rightSensor = nullptr;
static Adafruit_TMP117* s_leftSensor = nullptr;

// Static member definitions
int8_t TemperatureSensor::s_mainBoardTemp = INT8_MIN;
int8_t TemperatureSensor::s_rightBoardTemp = INT8_MIN;
int8_t TemperatureSensor::s_leftBoardTemp = INT8_MIN;

bool TemperatureSensor::s_mainBoardInit = false;
bool TemperatureSensor::s_rightBoardInit = false;
bool TemperatureSensor::s_leftBoardInit = false;

unsigned long TemperatureSensor::s_lastReadTime = 0;

// Per-sensor last successful read timestamps (0 = never read successfully)
unsigned long TemperatureSensor::s_mainLastSuccessMs = 0;
unsigned long TemperatureSensor::s_rightLastSuccessMs = 0;
unsigned long TemperatureSensor::s_leftLastSuccessMs = 0;

// Rotating sensor index - read one sensor per update cycle to avoid I2C bus contention
static uint8_t s_currentSensorIndex = 0;

// Configure a TMP117 sensor for fast reads (1X averaging = ~15.5ms conversion time)
static void configureSensorForFastReads(Adafruit_TMP117* sensor) {
    // Use single sample (no averaging) for fastest conversion (~15.5ms)
    // This is fine for board temperature monitoring where high precision isn't needed
    sensor->setAveragedSampleCount(TMP117_AVERAGE_1X);
    
    // Continuous mode (default) - sensor continuously takes measurements
    sensor->setMeasurementMode(TMP117_MODE_CONTINUOUS);
}

bool TemperatureSensor::init() {
    Logger::info(MODULE, "Initializing TMP117 temperature sensors...");
    
    // Initialize main board sensor (Wire, address 0x49)
    s_mainSensor = new Adafruit_TMP117();
    if (s_mainSensor->begin(TMP117_ADDR_MAIN, &Wire)) {
        s_mainBoardInit = true;
        configureSensorForFastReads(s_mainSensor);
        Logger::infof(MODULE, "Main board TMP117 (0x%02X on Wire) initialized", TMP117_ADDR_MAIN);
    } else {
        Logger::warningf(MODULE, "Main board TMP117 (0x%02X on Wire) not found", TMP117_ADDR_MAIN);
        delete s_mainSensor;
        s_mainSensor = nullptr;
    }
    
    // Initialize right board sensor (Wire, address 0x48)
    s_rightSensor = new Adafruit_TMP117();
    if (s_rightSensor->begin(TMP117_ADDR_RIGHT, &Wire)) {
        s_rightBoardInit = true;
        configureSensorForFastReads(s_rightSensor);
        Logger::infof(MODULE, "Right board TMP117 (0x%02X on Wire) initialized", TMP117_ADDR_RIGHT);
    } else {
        Logger::warningf(MODULE, "Right board TMP117 (0x%02X on Wire) not found", TMP117_ADDR_RIGHT);
        delete s_rightSensor;
        s_rightSensor = nullptr;
    }
    
    // Initialize left board sensor (SharedI2C, address 0x48)
    s_leftSensor = new Adafruit_TMP117();
    TwoWire& sharedBus = SharedI2C::getBus();
    if (s_leftSensor->begin(TMP117_ADDR_LEFT, &sharedBus)) {
        s_leftBoardInit = true;
        configureSensorForFastReads(s_leftSensor);
        Logger::infof(MODULE, "Left board TMP117 (0x%02X on SharedI2C) initialized", TMP117_ADDR_LEFT);
    } else {
        Logger::warningf(MODULE, "Left board TMP117 (0x%02X on SharedI2C) not found", TMP117_ADDR_LEFT);
        delete s_leftSensor;
        s_leftSensor = nullptr;
    }
    
    // Initialize success timestamps for sensors that passed init
    unsigned long now = millis();
    if (s_mainBoardInit) s_mainLastSuccessMs = now;
    if (s_rightBoardInit) s_rightLastSuccessMs = now;
    if (s_leftBoardInit) s_leftLastSuccessMs = now;
    
    // Report summary
    int sensorsFound = (s_mainBoardInit ? 1 : 0) + (s_rightBoardInit ? 1 : 0) + (s_leftBoardInit ? 1 : 0);
    
    if (sensorsFound == 0) {
        Logger::warning(MODULE, "No temperature sensors found! Temperature readings will report INT8_MIN");
    } else if (sensorsFound < 3) {
        Logger::warningf(MODULE, "Only %d/3 temperature sensors found, continuing with partial coverage", sensorsFound);
    } else {
        Logger::info(MODULE, "All 3 temperature sensors initialized successfully");
    }
    
    // When TEMP_SENSOR_REQUIRED is true, require all 3 sensors to be present
    // When false, always return true (warnings only, robot operates without temp protection)
    if (TEMP_SENSOR_REQUIRED) {
        if (sensorsFound < 3) {
            Logger::errorf(MODULE, "TEMP_SENSOR_REQUIRED: only %d/3 sensors found, halting", sensorsFound);
            return false;
        }
    }
    
    return true;
}

void TemperatureSensor::update() {
    unsigned long now = millis();
    
    // Read one sensor per interval (staggered to avoid I2C bus contention)
    // With 3 sensors and 1000ms interval, each sensor is read every ~3 seconds
    if (now - s_lastReadTime < TEMP_SENSOR_READ_INTERVAL_MS) {
        return;
    }
    
    s_lastReadTime = now;
    
    sensors_event_t temp;
    
    // Read only ONE sensor per update cycle, rotating through them
    // This prevents I2C bus contention issues
    // Use tryLock() for atomic check-and-acquire to avoid race conditions between cores
    switch (s_currentSensorIndex) {
        case 0:
            // Read main board sensor (Wire bus - shared with Motor2 encoder, IMU)
            if (s_mainSensor != nullptr) {
                // Atomically try to acquire the primary I2C bus lock
                if (SharedI2C::tryLockPrimary()) {
                    if (s_mainSensor->getEvent(&temp)) {
                        int8_t newTemp = (int8_t)roundf(temp.temperature);
                        // Sanity check: reject readings that are clearly wrong
                        // (e.g., exactly 0 when previous reading was significantly different)
                        if (newTemp != 0 || s_mainBoardTemp == INT8_MIN || abs(s_mainBoardTemp) <= 5) {
                            s_mainBoardTemp = newTemp;
                            s_mainLastSuccessMs = now;
                            Logger::debugf(MODULE, "Main board temp: %d°C", s_mainBoardTemp);
                        } else {
                            Logger::debugf(MODULE, "Main board temp: rejected suspicious 0°C (prev: %d°C)", s_mainBoardTemp);
                        }
                    } else {
                        Logger::warningf(MODULE, "Failed to read main board sensor (last success %lums ago)",
                                        s_mainLastSuccessMs > 0 ? (now - s_mainLastSuccessMs) : 0UL);
                    }
                    SharedI2C::unlockPrimary();
                } else {
                    Logger::debug(MODULE, "Primary I2C locked, skipping main board sensor read");
                }
            }
            break;
            
        case 1:
            // Read right board sensor (Wire bus - shared with Motor2 encoder, IMU)
            if (s_rightSensor != nullptr) {
                // Atomically try to acquire the primary I2C bus lock
                if (SharedI2C::tryLockPrimary()) {
                    if (s_rightSensor->getEvent(&temp)) {
                        int8_t newTemp = (int8_t)roundf(temp.temperature);
                        // Sanity check: reject readings that are clearly wrong
                        if (newTemp != 0 || s_rightBoardTemp == INT8_MIN || abs(s_rightBoardTemp) <= 5) {
                            s_rightBoardTemp = newTemp;
                            s_rightLastSuccessMs = now;
                            Logger::debugf(MODULE, "Right board temp: %d°C", s_rightBoardTemp);
                        } else {
                            Logger::debugf(MODULE, "Right board temp: rejected suspicious 0°C (prev: %d°C)", s_rightBoardTemp);
                        }
                    } else {
                        Logger::warningf(MODULE, "Failed to read right board sensor (last success %lums ago)",
                                        s_rightLastSuccessMs > 0 ? (now - s_rightLastSuccessMs) : 0UL);
                    }
                    SharedI2C::unlockPrimary();
                } else {
                    Logger::debug(MODULE, "Primary I2C locked, skipping right board sensor read");
                }
            }
            break;
            
        case 2:
            // Read left board sensor (SharedI2C bus - shared with Motor1 encoder, crypto)
            if (s_leftSensor != nullptr) {
                // Atomically try to acquire the SharedI2C bus lock
                if (SharedI2C::tryLock()) {
                    if (s_leftSensor->getEvent(&temp)) {
                        int8_t newTemp = (int8_t)roundf(temp.temperature);
                        // Sanity check: reject readings that are clearly wrong
                        if (newTemp != 0 || s_leftBoardTemp == INT8_MIN || abs(s_leftBoardTemp) <= 5) {
                            s_leftBoardTemp = newTemp;
                            s_leftLastSuccessMs = now;
                            Logger::debugf(MODULE, "Left board temp: %d°C", s_leftBoardTemp);
                        } else {
                            Logger::debugf(MODULE, "Left board temp: rejected suspicious 0°C (prev: %d°C)", s_leftBoardTemp);
                        }
                    } else {
                        Logger::warningf(MODULE, "Failed to read left board sensor (last success %lums ago)",
                                        s_leftLastSuccessMs > 0 ? (now - s_leftLastSuccessMs) : 0UL);
                    }
                    SharedI2C::unlock();
                } else {
                    // Bus is locked, skip this read but keep previous value
                    Logger::debug(MODULE, "SharedI2C locked, skipping left board sensor read");
                }
            }
            break;
    }
    
    // Rotate to next sensor
    s_currentSensorIndex = (s_currentSensorIndex + 1) % 3;
}

int8_t TemperatureSensor::getMainBoardTemp() {
    return s_mainBoardTemp;
}

int8_t TemperatureSensor::getRightBoardTemp() {
    return s_rightBoardTemp;
}

int8_t TemperatureSensor::getLeftBoardTemp() {
    return s_leftBoardTemp;
}

bool TemperatureSensor::isMainBoardInitialized() {
    return s_mainBoardInit;
}

bool TemperatureSensor::isRightBoardInitialized() {
    return s_rightBoardInit;
}

bool TemperatureSensor::isLeftBoardInitialized() {
    return s_leftBoardInit;
}

bool TemperatureSensor::isMainBoardOverheating() {
    // Only check if sensor is initialized and has a valid reading
    if (!s_mainBoardInit || s_mainBoardTemp == INT8_MIN) {
        return false;
    }
    return s_mainBoardTemp >= TEMP_LIMIT_MAIN_BOARD_CELSIUS;
}

bool TemperatureSensor::isMotorOverheating() {
    // Check right motor board (if initialized and has valid reading)
    if (s_rightBoardInit && s_rightBoardTemp != INT8_MIN) {
        if (s_rightBoardTemp >= TEMP_LIMIT_MOTOR_CELSIUS) {
            return true;
        }
    }
    
    // Check left motor board (if initialized and has valid reading)
    if (s_leftBoardInit && s_leftBoardTemp != INT8_MIN) {
        if (s_leftBoardTemp >= TEMP_LIMIT_MOTOR_CELSIUS) {
            return true;
        }
    }
    
    return false;
}

bool TemperatureSensor::hasSensorFailure() {
    unsigned long now = millis();
    
    // Sensor not initialized at boot = immediate failure
    if (!s_mainBoardInit || !s_rightBoardInit || !s_leftBoardInit) return true;
    
    // Sensor initialized but hasn't responded within TEMP_SENSOR_FAILURE_TIMEOUT_MS
    // A single missed read is OK (I2C bus can be busy), only sustained failures count
    if (s_mainLastSuccessMs > 0 && (now - s_mainLastSuccessMs) >= TEMP_SENSOR_FAILURE_TIMEOUT_MS) return true;
    if (s_rightLastSuccessMs > 0 && (now - s_rightLastSuccessMs) >= TEMP_SENSOR_FAILURE_TIMEOUT_MS) return true;
    if (s_leftLastSuccessMs > 0 && (now - s_leftLastSuccessMs) >= TEMP_SENSOR_FAILURE_TIMEOUT_MS) return true;
    
    return false;
}

bool TemperatureSensor::isOverheating() {
    // Always check actual temperature limits
    if (isMainBoardOverheating() || isMotorOverheating()) {
        return true;
    }
    
    // When TEMP_SENSOR_REQUIRED, also treat sensor failure as overheating
    // (prevents operating without thermal protection)
    if (TEMP_SENSOR_REQUIRED && hasSensorFailure()) {
        return true;
    }
    
    return false;
}

