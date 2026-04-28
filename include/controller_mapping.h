#ifndef CONTROLLER_MAPPING_H
#define CONTROLLER_MAPPING_H

#include <Arduino.h>
#include <Preferences.h>

// ============================================
// Controller Input IDs (what physical buttons/axes exist)
// ============================================

// Button inputs (0x01 - 0x1F)
#define CTRL_INPUT_NONE       0x00
#define CTRL_INPUT_A          0x01
#define CTRL_INPUT_B          0x02
#define CTRL_INPUT_X          0x03
#define CTRL_INPUT_Y          0x04
#define CTRL_INPUT_L1         0x05
#define CTRL_INPUT_R1         0x06
#define CTRL_INPUT_L2_BTN     0x07  // L2 as digital button
#define CTRL_INPUT_R2_BTN     0x08  // R2 as digital button
#define CTRL_INPUT_UP         0x09
#define CTRL_INPUT_DOWN       0x0A
#define CTRL_INPUT_LEFT       0x0B
#define CTRL_INPUT_RIGHT      0x0C
#define CTRL_INPUT_START      0x0D
#define CTRL_INPUT_SELECT     0x0E
#define CTRL_INPUT_L3         0x0F
#define CTRL_INPUT_R3         0x10
#define CTRL_INPUT_MISC       0x11
#define CTRL_INPUT_BTN_COUNT  17     // Total button inputs (excluding NONE)

// Axis inputs (0x20 - 0x2F)
#define CTRL_INPUT_AXIS_X     0x20  // Left stick X
#define CTRL_INPUT_AXIS_Y     0x21  // Left stick Y
#define CTRL_INPUT_AXIS_RX    0x22  // Right stick X
#define CTRL_INPUT_AXIS_RY    0x23  // Right stick Y
#define CTRL_INPUT_BRAKE      0x24  // L2 analog (0-1023)
#define CTRL_INPUT_THROTTLE   0x25  // R2 analog (0-1023)
#define CTRL_INPUT_AXIS_COUNT 6     // Total axis inputs

// ============================================
// Robot Action IDs (what the robot does)
// ============================================

// Bool/Digital actions (0x01 - 0x0F)
#define ROBOT_ACTION_NONE             0x00
#define ROBOT_ACTION_MOTORS_TOGGLE    0x01
#define ROBOT_ACTION_NEXT_EYE_COLOR   0x02
#define ROBOT_ACTION_STAND_UP         0x03
#define ROBOT_ACTION_PLAY_MUSIC       0x04
#define ROBOT_ACTION_SUBPROGRAM_1     0x05
#define ROBOT_ACTION_SUBPROGRAM_2     0x06
#define ROBOT_ACTION_SUBPROGRAM_3     0x07
#define ROBOT_ACTION_SUBPROGRAM_4     0x08
#define ROBOT_ACTION_SUBPROGRAM_5     0x09
#define ROBOT_ACTION_SCAN             0x0A
#define ROBOT_ACTION_PREV_EYE_COLOR   0x0B
#define ROBOT_ACTION_BOOL_COUNT       11   // Total bool actions (excluding NONE)

// Value/Analog actions (0x10 - 0x1F)
#define ROBOT_ACTION_MOVE             0x10
#define ROBOT_ACTION_TURN             0x11
#define ROBOT_ACTION_SPEED_BOOST      0x12
#define ROBOT_ACTION_VALUE_COUNT      3    // Total value actions

// ============================================
// Controller Types (from Bluepad32/Steam)
// ============================================

// Steam Controllers (1-29)
#define CONTROLLER_TYPE_UnknownSteamController      1
#define CONTROLLER_TYPE_SteamController             2
#define CONTROLLER_TYPE_SteamControllerV2           3

// Other Controllers (30+)
#define CONTROLLER_TYPE_UnknownNonSteamController   30
#define CONTROLLER_TYPE_XBox360Controller           31
#define CONTROLLER_TYPE_XBoxOneController           32
#define CONTROLLER_TYPE_PS3Controller               33
#define CONTROLLER_TYPE_PS4Controller               34
#define CONTROLLER_TYPE_WiiController               35
#define CONTROLLER_TYPE_AppleController             36
#define CONTROLLER_TYPE_AndroidController           37
#define CONTROLLER_TYPE_SwitchProController         38
#define CONTROLLER_TYPE_SwitchJoyConLeft            39
#define CONTROLLER_TYPE_SwitchJoyConRight           40
#define CONTROLLER_TYPE_SwitchJoyConPair            41
#define CONTROLLER_TYPE_SwitchInputOnlyController   42
#define CONTROLLER_TYPE_MobileTouch                 43
#define CONTROLLER_TYPE_XInputSwitchController      44
#define CONTROLLER_TYPE_PS5Controller               45

#define CONTROLLER_TYPE_MAX                         46  // For array sizing

// ============================================
// Mapping Structures
// ============================================

// Raw controller state (before mapping)
struct RawControllerInputs {
    bool isConnected;
    int controllerType;
    
    // Buttons
    bool a, b, x, y;
    bool l1, r1, l2Btn, r2Btn;
    bool up, down, left, right;
    bool start, select;
    bool l3, r3;
    bool misc;
    
    // Axes
    int axisX, axisY;      // Left stick
    int axisRX, axisRY;    // Right stick
    int brake, throttle;   // L2/R2 analog
    
    unsigned long timestamp;
};

// Axis mapping configuration
struct AxisMapping {
    uint8_t sourceAxis;    // Which CTRL_INPUT_AXIS_* to use
    bool inverted;         // Multiply by -1
};

// Complete mapping for a controller type
struct ControllerTypeMapping {
    // For each button: which robot bool action does it trigger (ROBOT_ACTION_* or NONE)
    // Index by (CTRL_INPUT_* - 1) for buttons 0x01-0x11
    uint8_t buttonActions[CTRL_INPUT_BTN_COUNT];
    
    // For each value action: which axis and inversion
    AxisMapping moveMapping;       // ROBOT_ACTION_MOVE
    AxisMapping turnMapping;       // ROBOT_ACTION_TURN
    AxisMapping speedBoostMapping; // ROBOT_ACTION_SPEED_BOOST
};

// ============================================
// ControllerMapping Class
// ============================================

class ControllerMapping {
public:
    static void init();
    
    // Get current mapping for a controller type
    static bool getMapping(uint8_t controllerType, ControllerTypeMapping& mapping);
    
    // Set custom mapping for a controller type (saved to preferences)
    static bool setMapping(uint8_t controllerType, const ControllerTypeMapping& mapping);
    
    // Reset to default mapping for a controller type
    static bool resetToDefault(uint8_t controllerType);
    
    // Check if a custom mapping exists for a controller type
    static bool hasCustomMapping(uint8_t controllerType);
    
    // Apply mapping: transform raw inputs to robot inputs
    // Returns button states and axis values for robot actions
    static void applyMapping(const RawControllerInputs& raw,
                            bool& motorsToggle, bool& nextEyeColor, bool& prevEyeColor, bool& standUp,
                            bool& playMusic, bool& subprogram1, bool& subprogram2,
                            bool& subprogram3, bool& subprogram4, bool& subprogram5,
                            bool& scan, int& moveAxis, int& turnAxis, int& speedBoostAxis);
    
    // Get default mapping for a controller type
    static void getDefaultMapping(uint8_t controllerType, ControllerTypeMapping& mapping);
    
private:
    static Preferences preferences;
    
    // Load mapping from preferences (returns false if not found)
    static bool loadMapping(uint8_t controllerType, ControllerTypeMapping& mapping);
    
    // Save mapping to preferences
    static void saveMapping(uint8_t controllerType, const ControllerTypeMapping& mapping);
    
    // Delete custom mapping from preferences
    static void deleteMapping(uint8_t controllerType);
    
    // Get axis value from raw inputs by axis ID
    static int getAxisValue(const RawControllerInputs& raw, uint8_t axisId);
    
    // Get button value from raw inputs by button ID
    static bool getButtonValue(const RawControllerInputs& raw, uint8_t buttonId);
    
    // Create preferences key for a controller type
    static void makeKey(uint8_t controllerType, char* key, size_t keyLen);
};

#endif // CONTROLLER_MAPPING_H

