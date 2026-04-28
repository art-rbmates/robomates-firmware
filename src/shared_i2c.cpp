#include "shared_i2c.h"
#include "logger.h"
#include "config.h"
#include <Arduino.h>

static const char* MODULE = "SharedI2C";

// Static member definitions
TwoWire SharedI2C::I2C_Bus = TwoWire(1);
bool SharedI2C::initialized = false;

// ESP32 spinlocks for proper multi-core synchronization
// volatile bool is NOT sufficient - cores have separate caches and volatile doesn't
// guarantee atomic operations or memory barriers between cores
static portMUX_TYPE s_sharedI2CMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_primaryI2CMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_busLocked = false;
static volatile bool s_primaryBusLocked = false;

bool SharedI2C::init() {
    if (initialized) {
        Logger::warning(MODULE, "Already initialized");
        return true;
    }
    
    Logger::infof(MODULE, "Initializing I2C bus: SDA=%d, SCL=%d, Clock=%dHz", 
                  SHARED_I2C_SDA_PIN, SHARED_I2C_SCL_PIN, SHARED_I2C_CLOCK);
    I2C_Bus.begin(SHARED_I2C_SDA_PIN, SHARED_I2C_SCL_PIN);
    I2C_Bus.setClock(SHARED_I2C_CLOCK);
    initialized = true;
    scanDevices();
    Logger::info(MODULE, "Initialized successfully");
    return true;
}

TwoWire& SharedI2C::getBus() {
    if (!initialized) {
        Logger::warning(MODULE, "Not initialized, calling init()");
        init();
    }
    return I2C_Bus;
}

void SharedI2C::scanDevices() {
    uint8_t found = 0;
    char devices[128] = "";
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        I2C_Bus.beginTransmission(addr);
        uint8_t error = I2C_Bus.endTransmission();
        
        if (error == 0) {
            char buf[8];
            snprintf(buf, sizeof(buf), "0x%02X ", addr);
            Logger::infof(MODULE, "I2C Device found: %s", buf);
            found++;
        }
    }
    
    if (found == 0) {
        Logger::info(MODULE, "I2C Device Scan: No devices found");
    }
}

bool SharedI2C::isInitialized() {
    return initialized;
}

void SharedI2C::lock() {
    portENTER_CRITICAL(&s_sharedI2CMux);
    s_busLocked = true;
    portEXIT_CRITICAL(&s_sharedI2CMux);
}

void SharedI2C::unlock() {
    portENTER_CRITICAL(&s_sharedI2CMux);
    s_busLocked = false;
    portEXIT_CRITICAL(&s_sharedI2CMux);
}

bool SharedI2C::isLocked() {
    portENTER_CRITICAL(&s_sharedI2CMux);
    bool locked = s_busLocked;
    portEXIT_CRITICAL(&s_sharedI2CMux);
    return locked;
}

bool SharedI2C::tryLock() {
    portENTER_CRITICAL(&s_sharedI2CMux);
    if (s_busLocked) {
        portEXIT_CRITICAL(&s_sharedI2CMux);
        return false;  // Already locked by another core
    }
    s_busLocked = true;
    portEXIT_CRITICAL(&s_sharedI2CMux);
    return true;  // Successfully acquired lock
}

void SharedI2C::lockPrimary() {
    portENTER_CRITICAL(&s_primaryI2CMux);
    s_primaryBusLocked = true;
    portEXIT_CRITICAL(&s_primaryI2CMux);
}

void SharedI2C::unlockPrimary() {
    portENTER_CRITICAL(&s_primaryI2CMux);
    s_primaryBusLocked = false;
    portEXIT_CRITICAL(&s_primaryI2CMux);
}

bool SharedI2C::isPrimaryLocked() {
    portENTER_CRITICAL(&s_primaryI2CMux);
    bool locked = s_primaryBusLocked;
    portEXIT_CRITICAL(&s_primaryI2CMux);
    return locked;
}

bool SharedI2C::tryLockPrimary() {
    portENTER_CRITICAL(&s_primaryI2CMux);
    if (s_primaryBusLocked) {
        portEXIT_CRITICAL(&s_primaryI2CMux);
        return false;  // Already locked by another core
    }
    s_primaryBusLocked = true;
    portEXIT_CRITICAL(&s_primaryI2CMux);
    return true;  // Successfully acquired lock
}
