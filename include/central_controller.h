#ifndef CENTRAL_CONTROLLER_H
#define CENTRAL_CONTROLLER_H

#include <Arduino.h>
#include "shared_data.h"

/**
 * CentralController - Unified controller input management
 * 
 * Priority system (highest to lowest):
 * 1. Subprogram control (handled in BalanceController via SubprogramControl)
 * 2. BLE controller input (from mobile app)
 * 3. Physical controller input (PS5/Bluetooth gamepad)
 * 
 * BLE controller input is only valid for BLE_CONTROLLER_INPUT_TIMEOUT_MS (100ms)
 * after receiving a packet. After timeout, falls back to physical controller.
 */
class CentralController {
public:
    // BLE controller input structure (raw values from mobile app)
    struct BleControllerInput {
        // Axes (-512 to 511 for sticks, 0 to 1023 for triggers)
        int16_t axisX;       // Left stick X
        int16_t axisY;       // Left stick Y (forward/backward)
        int16_t axisRX;      // Right stick X (turn)
        int16_t axisRY;      // Right stick Y
        int16_t l2Axis;      // L2 trigger (0-1023)
        int16_t r2Axis;      // R2 trigger (0-1023)
        
        // Buttons (bitmask for compact BLE transfer)
        bool a;              // Cross button
        bool b;              // Circle button
        bool x;              // Square button
        bool y;              // Triangle button
        bool l1;             // L1 shoulder
        bool r1;             // R1 shoulder
        bool up;             // D-pad up
        bool down;           // D-pad down
        bool left;           // D-pad left
        bool right;          // D-pad right
        
        uint32_t timestamp;  // When this input was received
    };
    
    // Initialize central controller
    static void init();
    
    // Set BLE controller input (called from BLEServer when packet received)
    static void setBleControllerInput(const BleControllerInput& input);
    
    // Check if BLE controller input is currently active (within timeout)
    static bool isBleControllerActive();
    
    // Get the effective robot inputs with priority handling
    // This should be called instead of SharedData::getRobotInputs()
    // Priority: BLE controller (if active) > Physical controller
    // Note: Subprogram override is still handled separately in BalanceController
    static SharedData::RobotInputs getRobotInputs();
    
    // Get current BLE controller input (for debugging/status)
    static BleControllerInput getBleControllerInput();
    
    // Get timestamp of last BLE controller input
    static uint32_t getLastBleInputTimestamp();
    
private:
    static BleControllerInput bleInput;
    static bool bleInputValid;  // Set to true when any BLE input received
    
    // Convert BLE controller input to RobotInputs (apply mapping)
    static SharedData::RobotInputs bleInputToRobotInputs(const BleControllerInput& input);
};

#endif // CENTRAL_CONTROLLER_H

