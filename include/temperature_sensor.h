#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <Arduino.h>

// Temperature sensor module for TMP117 sensors
// Three sensors:
// - Main board: Wire(19, 23), address 0x49
// - Right board: Wire(19, 23), address 0x48
// - Left board: SharedI2C (Wire1), address 0x48

class TemperatureSensor {
public:
    // Initialize all temperature sensors
    static bool init();
    
    // Periodic update - reads sensors at configured interval
    static void update();
    
    // Get temperature values (rounded to whole Celsius degrees)
    // Returns INT8_MIN (-128) if sensor is not available
    static int8_t getMainBoardTemp();     // Main board sensor (0x49 on Wire)
    static int8_t getRightBoardTemp();    // Right board sensor (0x48 on Wire)
    static int8_t getLeftBoardTemp();     // Left board sensor (0x48 on SharedI2C)
    
    // Check if sensors are initialized
    static bool isMainBoardInitialized();
    static bool isRightBoardInitialized();
    static bool isLeftBoardInitialized();
    
    // Thermal protection - check if any temperature exceeds limits
    static bool isOverheating();
    static bool isMainBoardOverheating();
    static bool isMotorOverheating();
    
    // Check if all sensors are initialized and responding
    // Returns true if any sensor is missing or has stopped providing readings
    static bool hasSensorFailure();

private:
    static int8_t s_mainBoardTemp;
    static int8_t s_rightBoardTemp;
    static int8_t s_leftBoardTemp;
    
    static bool s_mainBoardInit;
    static bool s_rightBoardInit;
    static bool s_leftBoardInit;
    
    static unsigned long s_lastReadTime;
    
    // Per-sensor last successful read timestamps (for runtime failure detection)
    static unsigned long s_mainLastSuccessMs;
    static unsigned long s_rightLastSuccessMs;
    static unsigned long s_leftLastSuccessMs;
};

#endif // TEMPERATURE_SENSOR_H

