#include "config.h"
#include "calibration_manager.h"
#include "imu.h"
#include "logger.h"
#include <Arduino.h>

static const char* MODULE = "Calib";

// Static member definitions
Preferences CalibrationManager::preferences;

void CalibrationManager::init() {
    preferences.begin("robot-calib", false);
    Logger::info(MODULE, "Preferences initialized");
}

bool CalibrationManager::loadCalibration(BLDCMotor& motor1, BLDCMotor& motor2) {
    if (RESET_CALIBRATION_FLAG) {
        Logger::warning(MODULE, "Resetting calibration data...");
        preferences.clear();
        return false;
    }

    // Check if all required calibration keys are present
    if (!preferences.isKey("calibrated")) {
        Logger::info(MODULE, "No calibration data found");
        return false;
    }

    // Verify all motor calibration parameters exist
    if (!preferences.isKey("m1_dir") || !preferences.isKey("m1_zero") ||
        !preferences.isKey("m2_dir") || !preferences.isKey("m2_zero")) {
        Logger::warning(MODULE, "Incomplete motor calibration data. Missing parameters.");
        preferences.clear();
        return false;
    }

    // Verify all IMU calibration parameters exist
    if (!preferences.isKey("imu_x") || !preferences.isKey("imu_y") || !preferences.isKey("imu_z")) {
        Logger::warning(MODULE, "Incomplete IMU calibration data. Missing parameters.");
        preferences.clear();
        return false;
    }

    Logger::info(MODULE, "Calibration data found. Loading...");
    
    // Load motor calibration data
    motor1.sensor_direction = (Direction)preferences.getUChar("m1_dir");
    motor1.zero_electric_angle = preferences.getFloat("m1_zero");
    motor2.sensor_direction = (Direction)preferences.getUChar("m2_dir");
    motor2.zero_electric_angle = preferences.getFloat("m2_zero");

    // Load IMU calibration data
    int16_t imu_x = preferences.getShort("imu_x");
    int16_t imu_y = preferences.getShort("imu_y");
    int16_t imu_z = preferences.getShort("imu_z");
    IMU::init(imu_x, imu_y, imu_z);
    
    Logger::info(MODULE, "Calibration loaded successfully");
    return true;
}

void CalibrationManager::saveCalibration(const BLDCMotor& motor1, const BLDCMotor& motor2) {
    Logger::info(MODULE, "Calibration complete. Saving data...");
    
    // Save motor calibration data
    preferences.putUChar("m1_dir", motor1.sensor_direction);
    preferences.putFloat("m1_zero", motor1.zero_electric_angle);
    preferences.putUChar("m2_dir", motor2.sensor_direction);
    preferences.putFloat("m2_zero", motor2.zero_electric_angle);
    
    // Save IMU calibration data
    preferences.putShort("imu_x", IMU::getXGyroOffset());
    preferences.putShort("imu_y", IMU::getYGyroOffset());
    preferences.putShort("imu_z", IMU::getZGyroOffset());
    
    preferences.putBool("calibrated", true);
    Logger::info(MODULE, "Calibration data saved");
}

void CalibrationManager::resetCalibration() {
    preferences.clear();
    Logger::info(MODULE, "Calibration data cleared");
}
