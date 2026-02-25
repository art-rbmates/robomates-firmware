#ifndef MOTOR_HARDWARE_H
#define MOTOR_HARDWARE_H

#include <SimpleFOC.h>
#include <Wire.h>

class MotorHardware {
public:
    // Initialize motor hardware
    static void init();
    
    // Initialize motors (FOC calibration)
    static void initializeMotors();
    
    // Calibrate motors
    static void calibrateMotors();
    
    // Run motor FOC loop (must be called frequently)
    static void runMotorLoop();
    
    // Set motor targets
    static void setMotor1Target(float target);
    static void setMotor2Target(float target);
    static float getMotor1Target();
    static float getMotor2Target();
    
    // Set motor phase voltages
    static void setMotor1PhaseVoltage(float Uq, float Ud, float angle_el);
    static void setMotor2PhaseVoltage(float Uq, float Ud, float angle_el);
    
    // Get motor velocities
    static float getMotor1Velocity();
    static float getMotor2Velocity();
    static float getMotorsVelocityAvg();
    static float getMotorsVelocityDiff();
    
    // Get motor references (for calibration manager)
    static BLDCMotor& getMotor1();
    static BLDCMotor& getMotor2();
    
private:
    // Motor components
    static BLDCMotor motor1;
    static BLDCDriver3PWM driver1;
    static MagneticSensorI2C sensor1;
    
    static BLDCMotor motor2;
    static BLDCDriver3PWM driver2;
    static MagneticSensorI2C sensor2;
    
    static void setupMotorHardware();
};

#endif // MOTOR_HARDWARE_H

