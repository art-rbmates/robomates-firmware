#include "imu.h"
#include "logger.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <math.h>

// Module name for logging
static const char* MODULE = "IMU";

// Static IMU instance
static MPU6050 mpu(0x69);

// Static member initialization
bool IMU::imuReady = false;
uint8_t IMU::fifoBuffer[64] = {0};
uint16_t IMU::packetSize = 0;

// Local static variables
static uint8_t mpuIntStatus = 0;
static uint8_t devStatus = 0;
static uint16_t fifoCount = 0;

bool IMU::init(int16_t xGyroOffset, int16_t yGyroOffset, int16_t zGyroOffset) {
    Logger::info(MODULE, "Initializing I2C devices...");
    
    mpu.initialize();
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setSleepEnabled(false);

    Logger::info(MODULE, "Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    if (xGyroOffset == 0 && yGyroOffset == 0 && zGyroOffset == 0) {
        Logger::info(MODULE, "No offsets provided, starting calibration...");
        mpu.CalibrateGyro(6);
    } else {
        Logger::infof(MODULE, "Applying provided offsets: X=%d, Y=%d, Z=%d", 
                     xGyroOffset, yGyroOffset, zGyroOffset);
        mpu.setXGyroOffset(xGyroOffset);
        mpu.setYGyroOffset(yGyroOffset);
        mpu.setZGyroOffset(zGyroOffset);
    }

    mpu.PrintActiveOffsets();

    if (devStatus == 0) {
        Logger::info(MODULE, "Enabling DMP...");
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();
        imuReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        Logger::info(MODULE, "Adjusting DMP sensor fusion gain...");
        mpu.setMemoryBank(0);
        mpu.setMemoryStartAddress(0x60);
        mpu.writeMemoryByte(0);
        mpu.writeMemoryByte(0x20);
        mpu.writeMemoryByte(0);
        mpu.writeMemoryByte(0);
        
        Logger::info(MODULE, "IMU initialized successfully");
        return true;
    } else {
        Logger::errorf(MODULE, "DMP Initialization failed (code %d)", devStatus);
        return false;
    }
}

bool IMU::hasData() {
    return imuReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}

float IMU::getPitchIMU() {
    static float pitch;
    Quaternion q;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    
    float sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    float cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
    
    return pitch;
}

float IMU::getPitch() {
    return getPitchIMU();
}

float IMU::getRollIMU() {
    static float roll;
    Quaternion q;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    return roll;
}

float IMU::getRoll() {
    return getRollIMU();
}

float IMU::getPitchFull() {
    const float pitchHalf = getPitchIMU();
    const float roll = getRollIMU();

    float pitchFull;

    if (std::fabs(roll) > M_PI_2) {
        pitchFull = (pitchHalf >= 0.0f)
                    ? M_PI - pitchHalf
                    : -M_PI - pitchHalf;
    } else {
        pitchFull = pitchHalf;
    }

    pitchFull = std::remainderf(pitchFull, 2.0f * M_PI);

    return pitchFull;
}

int16_t IMU::getXGyroOffset() {
    return mpu.getXGyroOffset();
}

int16_t IMU::getYGyroOffset() {
    return mpu.getYGyroOffset();
}

int16_t IMU::getZGyroOffset() {
    return mpu.getZGyroOffset();
}

bool IMU::isReady() {
    return imuReady;
}

