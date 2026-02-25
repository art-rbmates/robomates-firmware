#ifndef SHARED_DATA_H
#define SHARED_DATA_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class SharedData {
public:
    static void init();
    
    // Robot inputs after controller mapping is applied
    struct RobotInputs {
        bool isConnected;
        
        // Bool actions (trigger on button press)
        bool motorsToggle;      // Toggle motors on/off
        bool nextEyeColor;      // Cycle to next eye color
        bool prevEyeColor;      // Cycle to previous eye color
        bool standUp;           // Stand up / reset PIDs
        bool playMusic;         // Play melody
        bool subprogram1;       // Run subprogram 1
        bool subprogram2;       // Run subprogram 2
        bool subprogram3;       // Run subprogram 3
        bool subprogram4;       // Run subprogram 4
        bool subprogram5;       // Run subprogram 5
        bool scan;              // Proximity scan
        
        // Value actions (axis values)
        int moveAxis;           // Forward/backward movement (-512 to 511)
        int turnAxis;           // Left/right turning (-512 to 511)
        int speedBoostAxis;     // Speed boost (0 to 1023)
        
        unsigned long timestamp;
    };
    
    struct MotorState {
        float motor1Velocity;
        float motor2Velocity;
        float currentPitch;
        float currentRoll;
        bool motorsEnabled;
        bool hasFallen;
        unsigned long timestamp;
    };
    
    struct SystemState {
        uint16_t batteryMillivolts;
        bool centralConnected;
        float speedCoefficient;
        float torqueCoefficient;
    };
    
    // Subprogram control override
    struct SubprogramControl {
        bool active;          // If true, subprogram controls the robot
        int axisY;            // Forward/backward (-512 to 511)
        int axisRX;           // Rotation (-512 to 511)
    };
    
    static void setRobotInputs(const RobotInputs& inputs);
    static RobotInputs getRobotInputs();
    static void setMotorState(const MotorState& state);
    static MotorState getMotorState();
    static void setSystemState(const SystemState& state);
    static SystemState getSystemState();
    
    static void setSubprogramControl(const SubprogramControl& control);
    static SubprogramControl getSubprogramControl();
    
private:
    static SemaphoreHandle_t mutex;
    static RobotInputs robotInputs;
    static MotorState motorState;
    static SystemState systemState;
    static SubprogramControl subprogramControl;
};

#endif // SHARED_DATA_H

