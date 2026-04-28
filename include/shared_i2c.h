#ifndef SHARED_I2C_H
#define SHARED_I2C_H

#include <Wire.h>

// Shared I2C configuration for the entire system
// Two I2C buses:
//   - SharedI2C (Wire1, pins 21/22): Crypto chip, Motor1 encoder, Left temp sensor
//   - PrimaryI2C (Wire, pins 19/23): IMU, Motor2 encoder, Main/Right temp sensors
class SharedI2C {
public:
    // Initialize the shared I2C bus (call once at startup)
    static bool init();
    
    // Get reference to the shared I2C bus (Wire1)
    static TwoWire& getBus();
    
    // I2C scan utility function
    static void scanDevices();
    
    // Check if I2C is initialized
    static bool isInitialized();
    
    // === SharedI2C (Wire1) Mutex ===
    // For exclusive access when crypto chip operations are in progress
    // Lock the bus for exclusive access (blocks motor1 sensor reads)
    static void lock();
    
    // Unlock the bus
    static void unlock();
    
    // Check if bus is currently locked (non-blocking check for motor loop)
    static bool isLocked();
    
    // Atomically try to acquire lock - returns true if lock acquired, false if already locked
    // Use this instead of isLocked()+lock() to avoid race conditions between cores
    static bool tryLock();
    
    // === Primary I2C (Wire) Mutex ===
    // For exclusive access to the primary I2C bus (pins 19/23)
    // Used by: IMU, Motor2 encoder, Main temp sensor, Right temp sensor
    static void lockPrimary();
    static void unlockPrimary();
    static bool isPrimaryLocked();
    
    // Atomically try to acquire primary lock - returns true if acquired, false if already locked
    static bool tryLockPrimary();

private:
    static TwoWire I2C_Bus;
    static bool initialized;
    // Note: bus lock state is managed internally using ESP32 spinlocks
    // for proper multi-core synchronization (volatile bool is not sufficient)
    
    // I2C pin configuration
    static const int SDA_PIN = 21;
    static const int SCL_PIN = 22;
    static const uint32_t I2C_FREQUENCY = 100000; // 100kHz, 400kHz is too fast for crypto chip
};

#endif // SHARED_I2C_H
