#include "balance_controller.h"
#include "config.h"
#include "logger.h"
#include "motor_hardware.h"
#include "imu.h"
#include "shared_data.h"
#include "central_controller.h"
#include "mymath.h"
#include "melody_player.h"
#include "led_controller.h"
#include "temperature_sensor.h"
#include "adc.h"

static const char* MODULE = "Balance";

// Static member definitions
MyPIDController BalanceController::pid_stb     (PID_STB_P,      PID_STB_I,      PID_STB_D,      PID_STB_RAMP,      PID_STB_INTEGRAL_LIMIT,      PID_STB_LIMIT);
MyPIDController BalanceController::pid_steering(PID_STEERING_P, PID_STEERING_I, PID_STEERING_D, PID_STEERING_RAMP, PID_STEERING_INTEGRAL_LIMIT, PID_STEERING_LIMIT);
MyPIDController BalanceController::pid_vel     (PID_VEL_P,      PID_VEL_I,      PID_VEL_D,      PID_VEL_RAMP,      PID_VEL_INTEGRAL_LIMIT,      PID_VEL_LIMIT);
MyPIDController BalanceController::pid_crawl   (PID_CRAWL_P,    PID_CRAWL_I,    PID_CRAWL_D,    PID_CRAWL_RAMP,    PID_CRAWL_INTEGRAL_LIMIT,    PID_CRAWL_LIMIT);

LowPassFilter BalanceController::lpf_pid_steering(LPF_PID_STEERING_TF);
LowPassFilter BalanceController::lpf_vel_ctr(LPF_VEL_CTR_TF);

// Base values for speed coefficient calculations
const float BalanceController::base_max_crawl_velocity = BASE_MAX_CRAWL_VELOCITY;
float BalanceController::base_max_balance_velocity_forward = BASE_MAX_BALANCE_VELOCITY_FORWARD;
float BalanceController::base_max_boost_velocity_forward = BASE_MAX_BOOST_VELOCITY_FORWARD;
const float BalanceController::base_max_balance_velocity_backward = BASE_MAX_BALANCE_VELOCITY_BACKWARD;
float BalanceController::base_min_steering_sensitivity = BASE_MIN_STEERING_SENSITIVITY;
float BalanceController::base_max_steering_sensitivity = BASE_MAX_STEERING_SENSITIVITY;
float BalanceController::speed_coefficient = 1.0;
float BalanceController::torque_coefficient = 1.0;
bool BalanceController::motors_enabled = false;

float BalanceController::target_pitch_constant = DEFAULT_TARGET_PITCH_CONSTANT;
float BalanceController::max_target_pitch_deviation_constant = MAX_TARGET_PITCH_DEVIATION_CONSTANT;
float BalanceController::max_crawl_velocity = base_max_crawl_velocity;
float BalanceController::max_balance_velocity_forward = base_max_balance_velocity_forward * speed_coefficient;
float BalanceController::max_balance_velocity_backward = base_max_balance_velocity_backward * speed_coefficient;
float BalanceController::min_steering_sensitivity = base_min_steering_sensitivity * speed_coefficient;
float BalanceController::max_steering_sensitivity = base_max_steering_sensitivity * speed_coefficient;

bool BalanceController::fallen = false;
unsigned long BalanceController::last_fall_notification_ms = 0;
unsigned long BalanceController::fallConfirmStartMs = 0;
bool BalanceController::last_b_button_state = false;
Preferences BalanceController::preferences;

void BalanceController::init() {
    loadTargetPitchConstant();
    loadSteeringSensitivity();
    loadVelocityLimits();
    Logger::info(MODULE, "Balance controller initialized");
}

void BalanceController::loadTargetPitchConstant() {
    preferences.begin("robot-balance", false);
    
    if (preferences.isKey("target_pitch")) {
        target_pitch_constant = preferences.getFloat("target_pitch");
        Logger::infof(MODULE, "Loaded target_pitch_constant: %.2f", target_pitch_constant);
    } else {
        // Set default value and save it
        target_pitch_constant = DEFAULT_TARGET_PITCH_CONSTANT;
        preferences.putFloat("target_pitch", target_pitch_constant);
        Logger::infof(MODULE, "Initialized target_pitch_constant to default: %.2f", target_pitch_constant);
    }
}

void BalanceController::saveTargetPitchConstant() {
    preferences.putFloat("target_pitch", target_pitch_constant);
    Logger::infof(MODULE, "Saved target_pitch_constant: %.2f", target_pitch_constant);
}

void BalanceController::balanceTask(void* parameter) {
    Logger::info(MODULE, "Balance task running");
    MelodyPlayer::playStartupMelody(MotorHardware::getMotor1(), MotorHardware::getMotor2(), MelodyPlayer::getAmplitude(), MOTOR_VOLTAGE_LIMIT);
    while (true) {
        UBaseType_t hw = uxTaskGetStackHighWaterMark(nullptr);
        if (hw < 256) { // 256 bytes 
            Logger::warningf(MODULE, "Low balance stack: %u words (~%u bytes)", (unsigned)hw, (unsigned)(hw * 4));
        }
        update();
        MotorHardware::runMotorLoop();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void BalanceController::update() {
    // Read robot inputs via CentralController (handles BLE vs physical controller priority)
    // Priority: Subprogram (highest) > BLE controller > Physical controller (lowest)
    SharedData::RobotInputs inputs = CentralController::getRobotInputs();
    SharedData::SystemState system = SharedData::getSystemState();
    SharedData::SubprogramControl subprogramControl = SharedData::getSubprogramControl();
    
    // Override inputs if subprogram is active
    if (subprogramControl.active) {
        inputs.moveAxis = subprogramControl.axisY;
        inputs.turnAxis = subprogramControl.axisRX;
        // Disable other actions during subprogram execution
        inputs.standUp = false;
        inputs.playMusic = false;
        inputs.scan = false;
        inputs.speedBoostAxis = 0;
    }

    if (inputs.isConnected && inputs.playMusic && !BalanceController::last_b_button_state && !TemperatureSensor::isOverheating()) {
        MelodyPlayer::playMelody(MotorHardware::getMotor1(), MotorHardware::getMotor2(), MelodyPlayer::getAmplitude(), MOTOR_VOLTAGE_LIMIT);
    }
    last_b_button_state = inputs.playMusic;
    
    // Update speed and torque coefficients
    if (fabs(system.speedCoefficient - speed_coefficient) > 0.001f) {
        Logger::infof(MODULE, "Updating speed coefficient from %.3f to %.3f", speed_coefficient, system.speedCoefficient);
        setSpeedCoefficient(system.speedCoefficient);
    }
    if (fabs(system.torqueCoefficient - torque_coefficient) > 0.001f) {
        Logger::infof(MODULE, "Updating torque coefficient from %.3f to %.3f", torque_coefficient, system.torqueCoefficient);
        setTorqueCoefficient(system.torqueCoefficient);
    }
    
    // Handle motor enable/disable
    static bool lastMotorsToggleState = false;
    if (inputs.isConnected && inputs.motorsToggle && !lastMotorsToggleState) {
        motors_enabled = !motors_enabled;
        if (motors_enabled) {
            Logger::info(MODULE, "Motors enabled");
        } else {
            Logger::info(MODULE, "Motors disabled");
            MotorHardware::setMotor1Target(0);
            MotorHardware::setMotor2Target(0);
        }
    }
    lastMotorsToggleState = inputs.motorsToggle;
    
    // Thermal protection - disable motors if overheating or sensor failure
    static bool lastOverheatState = false;
    static bool lastSensorFailureState = false;
    bool actualOverheat = TemperatureSensor::isMainBoardOverheating() || TemperatureSensor::isMotorOverheating();
    bool sensorFailure = TEMP_SENSOR_REQUIRED && TemperatureSensor::hasSensorFailure();
    bool thermalShutdown = actualOverheat || sensorFailure;
    
    if (actualOverheat && !lastOverheatState) {
        // Just entered actual overheat state - log details
        if (TemperatureSensor::isMainBoardOverheating()) {
            Logger::errorf(MODULE, "THERMAL SHUTDOWN: Main board temp %d°C >= %d°C limit! Motors disabled.",
                           TemperatureSensor::getMainBoardTemp(), TEMP_LIMIT_MAIN_BOARD_CELSIUS);
        }
        if (TemperatureSensor::isMotorOverheating()) {
            Logger::errorf(MODULE, "THERMAL SHUTDOWN: Motor temp (L:%d°C R:%d°C) >= %d°C limit! Motors disabled.",
                           TemperatureSensor::getLeftBoardTemp(), TemperatureSensor::getRightBoardTemp(),
                           TEMP_LIMIT_MOTOR_CELSIUS);
        }
    } else if (actualOverheat) {
        // Still overheating - periodic warning every ~10s (balance runs at ~100Hz, 1000 iterations)
        static uint32_t overheatLogCounter = 0;
        if (++overheatLogCounter >= 1000) {
            overheatLogCounter = 0;
            Logger::warningf(MODULE, "Still overheating: main=%d°C, L=%d°C, R=%d°C",
                           TemperatureSensor::getMainBoardTemp(),
                           TemperatureSensor::getLeftBoardTemp(),
                           TemperatureSensor::getRightBoardTemp());
        }
    }
    lastOverheatState = actualOverheat;
    
    if (sensorFailure && !lastSensorFailureState) {
        // Just entered sensor failure state
        Logger::errorf(MODULE, "SENSOR FAILURE: Temperature sensor(s) not responding! Motors disabled. (init: M=%d R=%d L=%d)",
                      TemperatureSensor::isMainBoardInitialized(),
                      TemperatureSensor::isRightBoardInitialized(),
                      TemperatureSensor::isLeftBoardInitialized());
    } else if (sensorFailure) {
        // Still in sensor failure - periodic warning
        static uint32_t sensorFailLogCounter = 0;
        if (++sensorFailLogCounter >= 1000) {
            sensorFailLogCounter = 0;
            Logger::warningf(MODULE, "Still in sensor failure: temps M=%d°C R=%d°C L=%d°C",
                           TemperatureSensor::getMainBoardTemp(),
                           TemperatureSensor::getRightBoardTemp(),
                           TemperatureSensor::getLeftBoardTemp());
        }
    }
    lastSensorFailureState = sensorFailure;
    
    // Read IMU data once - used for both fall detection and balance control
    bool imuHasData = IMU::hasData();
    float pitch = 0;
    float roll = 0;
    
    // Fall detection runs ALWAYS (regardless of motor/controller/thermal state)
    // so that last_fall_ms_ago is always reported in pings
    if (imuHasData) {
        pitch = IMU::getPitchFull();
        roll = IMU::getRoll();
        bool isTilted = (fabs(pitch - target_pitch_constant) > max_target_pitch_deviation_constant) || speed_coefficient < 0.01f;
        
        if (isTilted) {
            if (fallConfirmStartMs == 0) {
                fallConfirmStartMs = millis();  // Start confirmation timer
            }
            // Only declare fallen after sustained tilt
            fallen = (millis() - fallConfirmStartMs >= FALL_CONFIRM_MS);
        } else {
            fallConfirmStartMs = 0;  // Reset timer
            fallen = false;
        }
    }
    
    bool controllerRequired = !inputs.isConnected;
    
    if (!motors_enabled || controllerRequired || thermalShutdown) {
        MotorHardware::setMotor1Target(0);
        MotorHardware::setMotor2Target(0);

        // White = actual overheating, Pink = sensor failure, Red = other disabled states
        if (actualOverheat) {
            LEDController::setLED(LED_L_BOT, CRGB::White);
            LEDController::setLED(LED_R_BOT, CRGB::White);
        } else if (sensorFailure) {
            LEDController::setLED(LED_L_BOT, CRGB(255, 0, 128));  // Pink
            LEDController::setLED(LED_R_BOT, CRGB(255, 0, 128));  // Pink
        } else {
            LEDController::setLED(LED_L_BOT, CRGB::Red);
            LEDController::setLED(LED_R_BOT, CRGB::Red);
        }
        
        // Update motor state in shared data
        SharedData::MotorState state;
        state.motor1Velocity = MotorHardware::getMotor1Velocity();
        state.motor2Velocity = MotorHardware::getMotor2Velocity();
        state.currentPitch = pitch;
        state.currentRoll = roll;
        state.motorsEnabled = false;
        state.hasFallen = fallen;  // Report actual fall state even when motors are off
        state.timestamp = millis();
        SharedData::setMotorState(state);
        return;
    }

    // When motors are enabled, show yellow bottom LEDs if battery is low, otherwise black
    static bool lastBatteryLowState = false;
    bool batteryLow = ADC::isBatteryLow();
    if (batteryLow && !lastBatteryLowState) {
        Logger::warningf(MODULE, "LOW BATTERY WARNING: %u mV - bottom LEDs set to yellow", ADC::batteryMillivolts());
    } else if (batteryLow) {
        // Periodic warning while battery is low (~every 10s)
        static uint32_t batteryLogCounter = 0;
        if (++batteryLogCounter >= 1000) {
            batteryLogCounter = 0;
            Logger::warningf(MODULE, "Battery still low: %u mV", ADC::batteryMillivolts());
        }
    } else if (!batteryLow && lastBatteryLowState) {
        Logger::infof(MODULE, "Battery recovered: %u mV", ADC::batteryMillivolts());
    }
    lastBatteryLowState = batteryLow;
    
    if (batteryLow) {
        LEDController::setLED(LED_L_BOT, CRGB::Yellow);
        LEDController::setLED(LED_R_BOT, CRGB::Yellow);
    } else {
        LEDController::setLED(LED_L_BOT, CRGB::Black);
        LEDController::setLED(LED_R_BOT, CRGB::Black);
    }
    
    // If no new IMU data, skip balance control (fall detection already handled above)
    if (!imuHasData) {
        return;
    }
    
    float motors_velocity_diff = MotorHardware::getMotorsVelocityDiff();
    float motors_velocity_avg = lpf_vel_ctr(MotorHardware::getMotorsVelocityAvg());
    
    // Update max balance velocity based on speed boost axis
    max_balance_velocity_forward = ((base_max_boost_velocity_forward - base_max_balance_velocity_forward) 
                                   * pow(inputs.speedBoostAxis / 1024.0, BASE_POWER_BOOST_AXIS_EXPONENT)
                                   + base_max_balance_velocity_forward) 
                                   * speed_coefficient;
    
    // Reset PID controllers when stand up is pressed and robot is tilted
    if (fabs(pitch - target_pitch_constant) > max_target_pitch_deviation_constant && inputs.standUp) {
        resetPIDControllers();
    }
    
    // Check if in crawl mode or balance mode
    // standUp button allows attempting balance even when tilted
    if ((fabs(pitch - target_pitch_constant) > max_target_pitch_deviation_constant && !inputs.standUp) || speed_coefficient < 0.01f) {
        // Crawl mode - robot is too tilted for balance (or speed disabled)
        
        float crawl_target_velocity = fmap(inputs.moveAxis, -512, 511, -max_crawl_velocity, max_crawl_velocity, false);
        float crawl_target_steering = -fmap(inputs.turnAxis, -512, 511, -max_crawl_velocity, max_crawl_velocity, false);
    
        float crawl_voltage_control = -pid_crawl(motors_velocity_avg - crawl_target_velocity);
        float crawl_steering_voltage_control = lpf_pid_steering(pid_steering(motors_velocity_diff - crawl_target_steering));
        
        float crawl_motor1_target = crawl_voltage_control - crawl_steering_voltage_control;
        float crawl_motor2_target = -crawl_voltage_control - crawl_steering_voltage_control;
        
        MotorHardware::setMotor1Target(crawl_motor1_target * torque_coefficient);
        MotorHardware::setMotor2Target(crawl_motor2_target * torque_coefficient);
    } else {
        // Balance mode (fallen state is set above based on pitch, not overridden here)
        
        // Steering calculation
        float steering_input = inputs.turnAxis / 512.0;
        float expo_steering = steering_input * abs(steering_input);
        float steering_sensitivity = fmap((long)(pow(abs(motors_velocity_avg) / max_balance_velocity_forward, 1.0 / 3.0) * max_balance_velocity_forward),
            0, (long)max_balance_velocity_forward, (long)max_steering_sensitivity, (long)min_steering_sensitivity, true);
        float target_steering = -expo_steering * steering_sensitivity;
        float steering_voltage_control = lpf_pid_steering(pid_steering(motors_velocity_diff - target_steering));
        
        // Velocity calculation
        float target_velocity = fmap(inputs.moveAxis, -512, 511, -max_balance_velocity_forward, max_balance_velocity_forward, false);
        
        if (target_velocity > 0) {
            target_velocity = max_balance_velocity_backward/max_balance_velocity_forward * target_velocity;
        }
        
        float target_pitch = target_pitch_constant + pid_vel(motors_velocity_avg - target_velocity);
        float balance_voltage_control = pid_stb(target_pitch - pitch);
        
        float balance_motor1_target = balance_voltage_control - steering_voltage_control;
        float balance_motor2_target = -balance_voltage_control - steering_voltage_control;
        
        MotorHardware::setMotor1Target(balance_motor1_target * torque_coefficient);
        MotorHardware::setMotor2Target(balance_motor2_target * torque_coefficient);
    }
    
    // Update motor state in shared data
    SharedData::MotorState state;
    state.motor1Velocity = MotorHardware::getMotor1Velocity();
    state.motor2Velocity = MotorHardware::getMotor2Velocity();
    state.currentPitch = pitch;
    state.currentRoll = roll;
    state.motorsEnabled = motors_enabled;
    state.hasFallen = fallen;
    state.timestamp = millis();
    SharedData::setMotorState(state);
}

void BalanceController::enableMotors() {
    motors_enabled = true;
    Logger::info(MODULE, "Motors enabled");
}

void BalanceController::disableMotors() {
    motors_enabled = false;
    MotorHardware::setMotor1Target(0);
    MotorHardware::setMotor2Target(0);
    Logger::info(MODULE, "Motors disabled");
}

bool BalanceController::areMotorsEnabled() {
    return motors_enabled;
}

void BalanceController::resetPIDControllers() {
    pid_stb.reset();
    pid_vel.reset();
    pid_steering.reset();
    pid_crawl.reset();
}

void BalanceController::setSpeedCoefficient(float coefficient) {
    Logger::infof(MODULE, "Setting speed coefficient to: %.3f", coefficient);
    if (coefficient < 0.0f) {
        Logger::infof(MODULE, "Speed coefficient is less than 0.0f, setting to 0.0f");
        coefficient = 0.0f;
    }
    if (coefficient > 1.0f) {
        Logger::infof(MODULE, "Speed coefficient is greater than 1.0f, setting to 1.0f");
        coefficient = 1.0f;
    }
    speed_coefficient = coefficient;
    
    max_balance_velocity_forward = base_max_balance_velocity_forward * speed_coefficient;
    max_balance_velocity_backward = base_max_balance_velocity_backward * speed_coefficient;
    max_crawl_velocity = base_max_crawl_velocity * speed_coefficient;
    min_steering_sensitivity = base_min_steering_sensitivity * speed_coefficient;
    max_steering_sensitivity = base_max_steering_sensitivity * speed_coefficient;
    
    Logger::infof(MODULE, "Speed coefficient set to: %.3f", speed_coefficient);
}

void BalanceController::setTorqueCoefficient(float coefficient) {
    Logger::infof(MODULE, "Setting torque coefficient to: %.3f", coefficient);
    if (coefficient < 0.0f) {
        Logger::infof(MODULE, "Torque coefficient is less than 0.0f, setting to 0.0f");
        coefficient = 0.0f;
    }
    if (coefficient > 1.0f) {
        Logger::infof(MODULE, "Torque coefficient is greater than 1.0f, setting to 1.0f");
        coefficient = 1.0f;
    }
    torque_coefficient = coefficient;
    Logger::infof(MODULE, "Torque coefficient set to: %.3f", torque_coefficient);
}

bool BalanceController::hasFallen() {
    return fallen;
}

float BalanceController::getTargetPitchConstant() {
    return target_pitch_constant;
}

void BalanceController::setTargetPitchConstant(float value) {
    target_pitch_constant = value;
    saveTargetPitchConstant();
    Logger::infof(MODULE, "Target pitch constant set to: %.2f", target_pitch_constant);
}

float BalanceController::getDefaultTargetPitchConstant() {
    return DEFAULT_TARGET_PITCH_CONSTANT;
}

void BalanceController::loadSteeringSensitivity() {
    // preferences already opened in loadTargetPitchConstant()
    
    if (preferences.isKey("min_steer_sens")) {
        base_min_steering_sensitivity = preferences.getFloat("min_steer_sens");
        Logger::infof(MODULE, "Loaded min_steering_sensitivity: %.2f", base_min_steering_sensitivity);
    } else {
        // Set default value and save it
        base_min_steering_sensitivity = BASE_MIN_STEERING_SENSITIVITY;
        preferences.putFloat("min_steer_sens", base_min_steering_sensitivity);
        Logger::infof(MODULE, "Initialized min_steering_sensitivity to default: %.2f", base_min_steering_sensitivity);
    }
    
    if (preferences.isKey("max_steer_sens")) {
        base_max_steering_sensitivity = preferences.getFloat("max_steer_sens");
        Logger::infof(MODULE, "Loaded max_steering_sensitivity: %.2f", base_max_steering_sensitivity);
    } else {
        // Set default value and save it
        base_max_steering_sensitivity = BASE_MAX_STEERING_SENSITIVITY;
        preferences.putFloat("max_steer_sens", base_max_steering_sensitivity);
        Logger::infof(MODULE, "Initialized max_steering_sensitivity to default: %.2f", base_max_steering_sensitivity);
    }
    
    // Update the derived values
    min_steering_sensitivity = base_min_steering_sensitivity * speed_coefficient;
    max_steering_sensitivity = base_max_steering_sensitivity * speed_coefficient;
}

void BalanceController::saveSteeringSensitivity() {
    preferences.putFloat("min_steer_sens", base_min_steering_sensitivity);
    preferences.putFloat("max_steer_sens", base_max_steering_sensitivity);
    Logger::infof(MODULE, "Saved steering sensitivity: min=%.2f, max=%.2f", 
                  base_min_steering_sensitivity, base_max_steering_sensitivity);
}

float BalanceController::getMinSteeringSensitivity() {
    return base_min_steering_sensitivity;
}

float BalanceController::getMaxSteeringSensitivity() {
    return base_max_steering_sensitivity;
}

void BalanceController::setSteeringSensitivity(float minValue, float maxValue) {
    base_min_steering_sensitivity = minValue;
    base_max_steering_sensitivity = maxValue;
    
    // Update derived values
    min_steering_sensitivity = base_min_steering_sensitivity * speed_coefficient;
    max_steering_sensitivity = base_max_steering_sensitivity * speed_coefficient;
    
    saveSteeringSensitivity();
    Logger::infof(MODULE, "Steering sensitivity set to: min=%.2f, max=%.2f", 
                  base_min_steering_sensitivity, base_max_steering_sensitivity);
}

float BalanceController::getDefaultMinSteeringSensitivity() {
    return BASE_MIN_STEERING_SENSITIVITY;
}

float BalanceController::getDefaultMaxSteeringSensitivity() {
    return BASE_MAX_STEERING_SENSITIVITY;
}

void BalanceController::loadVelocityLimits() {
    // preferences already opened in loadTargetPitchConstant()
    
    if (preferences.isKey("max_bal_vel_f")) {
        base_max_balance_velocity_forward = preferences.getFloat("max_bal_vel_f");
        Logger::infof(MODULE, "Loaded max_balance_velocity_forward: %.2f", base_max_balance_velocity_forward);
    } else {
        // Set default value and save it
        base_max_balance_velocity_forward = BASE_MAX_BALANCE_VELOCITY_FORWARD;
        preferences.putFloat("max_bal_vel_f", base_max_balance_velocity_forward);
        Logger::infof(MODULE, "Initialized max_balance_velocity_forward to default: %.2f", base_max_balance_velocity_forward);
    }
    
    if (preferences.isKey("max_boost_vel_f")) {
        base_max_boost_velocity_forward = preferences.getFloat("max_boost_vel_f");
        Logger::infof(MODULE, "Loaded max_boost_velocity_forward: %.2f", base_max_boost_velocity_forward);
    } else {
        // Set default value and save it
        base_max_boost_velocity_forward = BASE_MAX_BOOST_VELOCITY_FORWARD;
        preferences.putFloat("max_boost_vel_f", base_max_boost_velocity_forward);
        Logger::infof(MODULE, "Initialized max_boost_velocity_forward to default: %.2f", base_max_boost_velocity_forward);
    }
    
    // Update derived values
    max_balance_velocity_forward = base_max_balance_velocity_forward * speed_coefficient;
}

void BalanceController::saveVelocityLimits() {
    preferences.putFloat("max_bal_vel_f", base_max_balance_velocity_forward);
    preferences.putFloat("max_boost_vel_f", base_max_boost_velocity_forward);
    Logger::infof(MODULE, "Saved velocity limits: balance=%.2f, boost=%.2f", 
                  base_max_balance_velocity_forward, base_max_boost_velocity_forward);
}

float BalanceController::getMaxBalanceVelocityForward() {
    return base_max_balance_velocity_forward;
}

float BalanceController::getMaxBoostVelocityForward() {
    return base_max_boost_velocity_forward;
}

void BalanceController::setVelocityLimits(float balanceVelocity, float boostVelocity) {
    base_max_balance_velocity_forward = balanceVelocity;
    base_max_boost_velocity_forward = boostVelocity;
    
    // Update derived values
    max_balance_velocity_forward = base_max_balance_velocity_forward * speed_coefficient;
    
    saveVelocityLimits();
    Logger::infof(MODULE, "Velocity limits set to: balance=%.2f, boost=%.2f", 
                  base_max_balance_velocity_forward, base_max_boost_velocity_forward);
}

float BalanceController::getDefaultMaxBalanceVelocityForward() {
    return BASE_MAX_BALANCE_VELOCITY_FORWARD;
}

float BalanceController::getDefaultMaxBoostVelocityForward() {
    return BASE_MAX_BOOST_VELOCITY_FORWARD;
}

