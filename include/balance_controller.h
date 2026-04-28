#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <SimpleFOC.h>
#include <Preferences.h>
#include "mypid.h"

class BalanceController {
public:
    // Initialize balance controller
    static void init();
    
    // Balance control task function (called by FreeRTOS)
    static void balanceTask(void* parameter);
    
    // Main balance control loop (called from task)
    static void update();
    
    // Enable/disable motors
    static void enableMotors();
    static void disableMotors();
    static bool areMotorsEnabled();
    
    // Reset PID controllers
    static void resetPIDControllers();
    
    // Speed and torque coefficient control
    static void setSpeedCoefficient(float coefficient);
    static void setTorqueCoefficient(float coefficient);
    
    // Get current state
    static bool hasFallen();
    
    // Target pitch constant access (for BLE)
    static float getTargetPitchConstant();
    static void setTargetPitchConstant(float value);
    static float getDefaultTargetPitchConstant();
    
    // Steering sensitivity access (for BLE)
    static float getMinSteeringSensitivity();
    static float getMaxSteeringSensitivity();
    static void setSteeringSensitivity(float minValue, float maxValue);
    static float getDefaultMinSteeringSensitivity();
    static float getDefaultMaxSteeringSensitivity();
    
    // Velocity limits access (for BLE)
    static float getMaxBalanceVelocityForward();
    static float getMaxBoostVelocityForward();
    static void setVelocityLimits(float balanceVelocity, float boostVelocity);
    static float getDefaultMaxBalanceVelocityForward();
    static float getDefaultMaxBoostVelocityForward();
    
    // Preference persistence
    static void loadTargetPitchConstant();
    static void saveTargetPitchConstant();
    static void loadSteeringSensitivity();
    static void saveSteeringSensitivity();
    static void loadVelocityLimits();
    static void saveVelocityLimits();
    
private:
    static Preferences preferences;
    // PID Controllers
    static MyPIDController pid_stb;
    static MyPIDController pid_steering;
    static MyPIDController pid_vel;
    static MyPIDController pid_crawl;
    static LowPassFilter lpf_pid_steering;
    static LowPassFilter lpf_vel_ctr;
    
    // Control parameters
    static float target_pitch_constant;
    static float max_target_pitch_deviation_constant;
    static float max_crawl_velocity;
    static float max_balance_velocity_forward;
    static float max_balance_velocity_backward;
    static float min_steering_sensitivity;
    static float max_steering_sensitivity;
    static bool motors_enabled;
    
    // Base values for speed calculations
    static const float base_max_crawl_velocity;
    static float base_max_balance_velocity_forward;
    static float base_max_boost_velocity_forward;
    static const float base_max_balance_velocity_backward;
    static float base_min_steering_sensitivity;
    static float base_max_steering_sensitivity;
    static float speed_coefficient;
    static float torque_coefficient;
    
    // Fall detection
    static bool fallen;
    static unsigned long last_fall_notification_ms;
    static const unsigned long fall_notification_cooldown_ms = 500;
    static unsigned long fallConfirmStartMs;  // Debounce: timestamp when tilt first exceeded threshold (0 = not tilting)
    
    // Startup jingle
    static bool last_b_button_state;
};

#endif // BALANCE_CONTROLLER_H

