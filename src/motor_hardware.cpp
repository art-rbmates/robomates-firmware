#include "motor_hardware.h"
#include "logger.h"
#include "calibration_manager.h"
#include "shared_i2c.h"
#include "imu.h"
#include "config.h"

static const char* MODULE = "MotorHW";

BLDCMotor MotorHardware::motor1 = BLDCMotor(MOTOR_PP, MOTOR_R);
BLDCDriver3PWM MotorHardware::driver1 = BLDCDriver3PWM(MOTOR1_PHASE_A_PIN, MOTOR1_PHASE_B_PIN, MOTOR1_PHASE_C_PIN, MOTOR1_ENABLE_PIN); // left motor
MagneticSensorI2C MotorHardware::sensor1 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor MotorHardware::motor2 = BLDCMotor(MOTOR_PP, MOTOR_R);
BLDCDriver3PWM MotorHardware::driver2 = BLDCDriver3PWM(MOTOR2_PHASE_A_PIN, MOTOR2_PHASE_B_PIN, MOTOR2_PHASE_C_PIN, MOTOR2_ENABLE_PIN); // right motor
MagneticSensorI2C MotorHardware::sensor2 = MagneticSensorI2C(AS5600_I2C);

void MotorHardware::init() {
    Logger::info(MODULE, "Initializing motor hardware");
    setupMotorHardware();
}

void MotorHardware::setupMotorHardware() {
    sensor1.init(&SharedI2C::getBus());
    motor1.linkSensor(&sensor1);
    motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver1.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;
    driver1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver1.init();
    motor1.linkDriver(&driver1);
    motor1.controller = MotionControlType::torque;
    motor1.LPF_velocity.Tf = 0;
    motor1.useMonitoring(Serial);
    
    sensor2.init();
    motor2.linkSensor(&sensor2);
    motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver2.voltage_power_supply = MOTOR_VOLTAGE_LIMIT;
    driver2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver2.init();
    motor2.linkDriver(&driver2);
    motor2.controller = MotionControlType::torque;
    motor2.LPF_velocity.Tf = 0;
    motor2.useMonitoring(Serial);
    
    Logger::info(MODULE, "Motor hardware setup complete");

    logAS5600Diagnostics(SharedI2C::getBus(), "Motor1/Left");
    logAS5600Diagnostics(Wire, "Motor2/Right");
}

void MotorHardware::logAS5600Diagnostics(TwoWire& bus, const char* label) {
    const uint8_t AS5600_ADDR = 0x36;
    const uint8_t REG_STATUS    = 0x0B;
    const uint8_t REG_AGC       = 0x1A;
    const uint8_t REG_MAGNITUDE = 0x1B;

    auto readReg8 = [&](uint8_t reg) -> int16_t {
        bus.beginTransmission(AS5600_ADDR);
        bus.write(reg);
        if (bus.endTransmission(false) != 0) return -1;
        if (bus.requestFrom(AS5600_ADDR, (uint8_t)1) != 1) return -1;
        return bus.read();
    };

    auto readReg16 = [&](uint8_t reg) -> int32_t {
        bus.beginTransmission(AS5600_ADDR);
        bus.write(reg);
        if (bus.endTransmission(false) != 0) return -1;
        if (bus.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return -1;
        uint16_t hi = bus.read();
        uint16_t lo = bus.read();
        return ((hi & 0x0F) << 8) | lo;
    };

    int16_t status = readReg8(REG_STATUS);
    int16_t agc    = readReg8(REG_AGC);
    int32_t mag    = readReg16(REG_MAGNITUDE);

    if (status < 0 || agc < 0 || mag < 0) {
        Logger::warningf(MODULE, "AS5600 [%s]: I2C read failed (status=%d agc=%d mag=%ld)",
                         label, status, agc, mag);
        return;
    }

    bool md = (status >> 5) & 1;
    bool ml = (status >> 4) & 1;
    bool mh = (status >> 3) & 1;

    Logger::infof(MODULE, "AS5600 [%s]: STATUS=0x%02X (MD=%d ML=%d MH=%d) AGC=%d MAGNITUDE=%ld",
                  label, status, md, ml, mh, agc, mag);

    if (!md)
        Logger::warning(MODULE, "  -> No magnet detected!");
    if (ml)
        Logger::warning(MODULE, "  -> Magnet too WEAK (AGC at max gain)");
    if (mh)
        Logger::warning(MODULE, "  -> Magnet too STRONG (AGC at min gain)");
}

void MotorHardware::initializeMotors() {
    Logger::info(MODULE, "Initializing motors with calibration");
    CalibrationManager::init();
    
    if (CalibrationManager::loadCalibration(motor1, motor2)) {
        motor1.init();
        motor1.initFOC();
        motor2.init();
        motor2.initFOC();
        Logger::info(MODULE, "Motors initialized with saved calibration");
    } else {
        calibrateMotors();
    }
}

void MotorHardware::calibrateMotors() {
    Logger::info(MODULE, "No calibration data found. Starting calibration...");
    
    // Calibrate IMU first
    IMU::init(0, 0, 0);
    
    // Calibrate motors
    motor1.init();
    motor1.initFOC();
    motor2.init();
    motor2.initFOC();
    
    CalibrationManager::saveCalibration(motor1, motor2);
    Logger::info(MODULE, "Motor calibration complete and saved");
}

void MotorHardware::runMotorLoop() {
    // Skip motor1 if shared I2C bus is locked (crypto operations or temp sensor read)
    // Motor1's sensor uses SharedI2C, so we must not access it during those ops
    if (!SharedI2C::isLocked()) {
        motor1.loopFOC();
        motor1.move();
    }
    // Skip motor2 if primary I2C bus is locked (temp sensor read in progress)
    // Motor2's sensor uses Wire (primary I2C), shared with IMU and temp sensors
    if (!SharedI2C::isPrimaryLocked()) {
        motor2.loopFOC();
        motor2.move();
    }
}

void MotorHardware::setMotor1Target(float target) {
    motor1.target = target;
}

void MotorHardware::setMotor2Target(float target) {
    motor2.target = target;
}

void MotorHardware::setMotor1PhaseVoltage(float Uq, float Ud, float angle_el) {
    motor1.setPhaseVoltage(Uq, Ud, angle_el);
}

void MotorHardware::setMotor2PhaseVoltage(float Uq, float Ud, float angle_el) {
    motor2.setPhaseVoltage(Uq, Ud, angle_el);
}

float MotorHardware::getMotor1Target() {
    return motor1.target;
}

float MotorHardware::getMotor2Target() {
    return motor2.target;
}

float MotorHardware::getMotor1Velocity() {
    return motor1.shaft_velocity;
}

float MotorHardware::getMotor2Velocity() {
    return motor2.shaft_velocity;
}

float MotorHardware::getMotorsVelocityAvg() {
    return (motor1.shaft_velocity + -motor2.shaft_velocity) / 2.0f;
}

float MotorHardware::getMotorsVelocityDiff() {
    return motor1.shaft_velocity - -motor2.shaft_velocity;
}

BLDCMotor& MotorHardware::getMotor1() {
    return motor1;
}

BLDCMotor& MotorHardware::getMotor2() {
    return motor2;
}

