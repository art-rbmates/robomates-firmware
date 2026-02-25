#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <Wire.h>

class IMU {
public:
    // Initialize the IMU with gyro offsets
    static bool init(int16_t xGyroOffset, int16_t yGyroOffset, int16_t zGyroOffset);
    
    // Check if IMU has new data available
    static bool hasData();
    
    // Get current pitch angle (radians)
    static float getPitch();
    
    // Get full pitch angle without wrapping (radians)
    static float getPitchFull();
    
    // Get current roll angle (radians)
    static float getRoll();
    
    // Get gyro offsets (used for calibration)
    static int16_t getXGyroOffset();
    static int16_t getYGyroOffset();
    static int16_t getZGyroOffset();
    
    // Check if IMU is ready
    static bool isReady();
    
private:
    static bool imuReady;
    static uint8_t fifoBuffer[64];
    static uint16_t packetSize;
    
    // Helper functions
    static float getPitchIMU();
    static float getRollIMU();
};

#endif // IMU_H

