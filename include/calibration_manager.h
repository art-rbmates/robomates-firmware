#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <SimpleFOC.h>
#include <Preferences.h>

class CalibrationManager {
public:
    static void init();
    static bool loadCalibration(BLDCMotor& motor1, BLDCMotor& motor2);
    static void saveCalibration(const BLDCMotor& motor1, const BLDCMotor& motor2);
    static void resetCalibration();
    
private:
    static Preferences preferences;
};

#endif // CALIBRATION_MANAGER_H
