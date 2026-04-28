#include "controller_mapping.h"
#include "logger.h"
#include <cstring>

static const char* MODULE = "CtrlMap";

Preferences ControllerMapping::preferences;

void ControllerMapping::init() {
    preferences.begin("ctrl-mapping", false);
    Logger::info(MODULE, "Controller mapping initialized");
}

void ControllerMapping::makeKey(uint8_t controllerType, char* key, size_t keyLen) {
    snprintf(key, keyLen, "map_%u", controllerType);
}

bool ControllerMapping::loadMapping(uint8_t controllerType, ControllerTypeMapping& mapping) {
    char key[16];
    makeKey(controllerType, key, sizeof(key));
    
    if (!preferences.isKey(key)) {
        return false;
    }
    
    size_t len = preferences.getBytes(key, &mapping, sizeof(mapping));
    if (len != sizeof(mapping)) {
        Logger::warningf(MODULE, "Invalid mapping data for type %u", controllerType);
        return false;
    }
    
    return true;
}

void ControllerMapping::saveMapping(uint8_t controllerType, const ControllerTypeMapping& mapping) {
    char key[16];
    makeKey(controllerType, key, sizeof(key));
    
    preferences.putBytes(key, &mapping, sizeof(mapping));
    Logger::infof(MODULE, "Saved mapping for controller type %u", controllerType);
}

void ControllerMapping::deleteMapping(uint8_t controllerType) {
    char key[16];
    makeKey(controllerType, key, sizeof(key));
    
    if (preferences.isKey(key)) {
        preferences.remove(key);
        Logger::infof(MODULE, "Deleted custom mapping for type %u", controllerType);
    }
}

bool ControllerMapping::hasCustomMapping(uint8_t controllerType) {
    char key[16];
    makeKey(controllerType, key, sizeof(key));
    return preferences.isKey(key);
}

void ControllerMapping::getDefaultMapping(uint8_t controllerType, ControllerTypeMapping& mapping) {
    // Initialize all to NONE/default
    memset(&mapping, 0, sizeof(mapping));
    
    // Default axis mappings (same for most controllers)
    mapping.moveMapping.sourceAxis = CTRL_INPUT_AXIS_Y;
    mapping.moveMapping.inverted = false;
    
    mapping.turnMapping.sourceAxis = CTRL_INPUT_AXIS_RX;
    mapping.turnMapping.inverted = false;
    
    mapping.speedBoostMapping.sourceAxis = CTRL_INPUT_THROTTLE;
    mapping.speedBoostMapping.inverted = false;
    
    // Default button mappings based on controller type
    // Most controllers use similar layouts, but some differ
    
    switch (controllerType) {
        case CONTROLLER_TYPE_PS3Controller:
        case CONTROLLER_TYPE_PS4Controller:
        case CONTROLLER_TYPE_PS5Controller:
            // PlayStation layout: Cross=A, Circle=B, Square=X, Triangle=Y
            // Cross (A) = Stand up
            // Circle (B) = Music
            // Square (X) = Subprogram 1
            // Triangle (Y) = Subprogram 2
            // Subprograms 3, 4, 5 are not mapped by default
            mapping.buttonActions[CTRL_INPUT_A - 1] = ROBOT_ACTION_STAND_UP;
            mapping.buttonActions[CTRL_INPUT_B - 1] = ROBOT_ACTION_PLAY_MUSIC;
            mapping.buttonActions[CTRL_INPUT_X - 1] = ROBOT_ACTION_SUBPROGRAM_1;
            mapping.buttonActions[CTRL_INPUT_Y - 1] = ROBOT_ACTION_SUBPROGRAM_2;
            mapping.buttonActions[CTRL_INPUT_L1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_R1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_UP - 1] = ROBOT_ACTION_MOTORS_TOGGLE;
            mapping.buttonActions[CTRL_INPUT_LEFT - 1] = ROBOT_ACTION_PREV_EYE_COLOR;
            mapping.buttonActions[CTRL_INPUT_RIGHT - 1] = ROBOT_ACTION_NEXT_EYE_COLOR;
            break;
            
        case CONTROLLER_TYPE_SwitchProController:
        case CONTROLLER_TYPE_SwitchJoyConPair:
        case CONTROLLER_TYPE_SwitchInputOnlyController:
            // Nintendo layout: B=bottom, A=right, Y=left, X=top
            // But Bluepad32 normalizes to standard layout
            mapping.buttonActions[CTRL_INPUT_A - 1] = ROBOT_ACTION_STAND_UP;
            mapping.buttonActions[CTRL_INPUT_B - 1] = ROBOT_ACTION_PLAY_MUSIC;
            mapping.buttonActions[CTRL_INPUT_X - 1] = ROBOT_ACTION_SUBPROGRAM_1;
            mapping.buttonActions[CTRL_INPUT_Y - 1] = ROBOT_ACTION_SUBPROGRAM_2;
            mapping.buttonActions[CTRL_INPUT_L1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_R1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_UP - 1] = ROBOT_ACTION_MOTORS_TOGGLE;
            mapping.buttonActions[CTRL_INPUT_LEFT - 1] = ROBOT_ACTION_PREV_EYE_COLOR;
            mapping.buttonActions[CTRL_INPUT_RIGHT - 1] = ROBOT_ACTION_NEXT_EYE_COLOR;
            break;
            
        case CONTROLLER_TYPE_XBox360Controller:
        case CONTROLLER_TYPE_XBoxOneController:
        case CONTROLLER_TYPE_XInputSwitchController:
            // Xbox layout: A=bottom, B=right, X=left, Y=top
            mapping.buttonActions[CTRL_INPUT_A - 1] = ROBOT_ACTION_STAND_UP;
            mapping.buttonActions[CTRL_INPUT_B - 1] = ROBOT_ACTION_PLAY_MUSIC;
            mapping.buttonActions[CTRL_INPUT_X - 1] = ROBOT_ACTION_SUBPROGRAM_1;
            mapping.buttonActions[CTRL_INPUT_Y - 1] = ROBOT_ACTION_SUBPROGRAM_2;
            mapping.buttonActions[CTRL_INPUT_L1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_R1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_UP - 1] = ROBOT_ACTION_MOTORS_TOGGLE;
            mapping.buttonActions[CTRL_INPUT_LEFT - 1] = ROBOT_ACTION_PREV_EYE_COLOR;
            mapping.buttonActions[CTRL_INPUT_RIGHT - 1] = ROBOT_ACTION_NEXT_EYE_COLOR;
            break;
            
        default:
            // Generic/Unknown controllers - use standard mapping
            mapping.buttonActions[CTRL_INPUT_A - 1] = ROBOT_ACTION_STAND_UP;
            mapping.buttonActions[CTRL_INPUT_B - 1] = ROBOT_ACTION_PLAY_MUSIC;
            mapping.buttonActions[CTRL_INPUT_X - 1] = ROBOT_ACTION_SUBPROGRAM_1;
            mapping.buttonActions[CTRL_INPUT_Y - 1] = ROBOT_ACTION_SUBPROGRAM_2;
            mapping.buttonActions[CTRL_INPUT_L1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_R1 - 1] = ROBOT_ACTION_SCAN;
            mapping.buttonActions[CTRL_INPUT_UP - 1] = ROBOT_ACTION_MOTORS_TOGGLE;
            mapping.buttonActions[CTRL_INPUT_LEFT - 1] = ROBOT_ACTION_PREV_EYE_COLOR;
            mapping.buttonActions[CTRL_INPUT_RIGHT - 1] = ROBOT_ACTION_NEXT_EYE_COLOR;
            break;
    }
}

bool ControllerMapping::getMapping(uint8_t controllerType, ControllerTypeMapping& mapping) {
    // Try to load custom mapping first
    if (loadMapping(controllerType, mapping)) {
        // Logger::debugf(MODULE, "Using custom mapping for type %u", controllerType);
        return true;
    }
    
    // Fall back to default
    getDefaultMapping(controllerType, mapping);
    // Logger::debugf(MODULE, "Using default mapping for type %u", controllerType);
    return true;
}

bool ControllerMapping::setMapping(uint8_t controllerType, const ControllerTypeMapping& mapping) {
    if (controllerType == 0 || controllerType >= CONTROLLER_TYPE_MAX) {
        Logger::errorf(MODULE, "Invalid controller type: %u", controllerType);
        return false;
    }
    
    saveMapping(controllerType, mapping);
    return true;
}

bool ControllerMapping::resetToDefault(uint8_t controllerType) {
    if (controllerType == 0 || controllerType >= CONTROLLER_TYPE_MAX) {
        Logger::errorf(MODULE, "Invalid controller type: %u", controllerType);
        return false;
    }
    
    deleteMapping(controllerType);
    return true;
}

int ControllerMapping::getAxisValue(const RawControllerInputs& raw, uint8_t axisId) {
    switch (axisId) {
        case CTRL_INPUT_AXIS_X:   return raw.axisX;
        case CTRL_INPUT_AXIS_Y:   return raw.axisY;
        case CTRL_INPUT_AXIS_RX:  return raw.axisRX;
        case CTRL_INPUT_AXIS_RY:  return raw.axisRY;
        case CTRL_INPUT_BRAKE:    return raw.brake;
        case CTRL_INPUT_THROTTLE: return raw.throttle;
        default: return 0;
    }
}

bool ControllerMapping::getButtonValue(const RawControllerInputs& raw, uint8_t buttonId) {
    switch (buttonId) {
        case CTRL_INPUT_A:      return raw.a;
        case CTRL_INPUT_B:      return raw.b;
        case CTRL_INPUT_X:      return raw.x;
        case CTRL_INPUT_Y:      return raw.y;
        case CTRL_INPUT_L1:     return raw.l1;
        case CTRL_INPUT_R1:     return raw.r1;
        case CTRL_INPUT_L2_BTN: return raw.l2Btn;
        case CTRL_INPUT_R2_BTN: return raw.r2Btn;
        case CTRL_INPUT_UP:     return raw.up;
        case CTRL_INPUT_DOWN:   return raw.down;
        case CTRL_INPUT_LEFT:   return raw.left;
        case CTRL_INPUT_RIGHT:  return raw.right;
        case CTRL_INPUT_START:  return raw.start;
        case CTRL_INPUT_SELECT: return raw.select;
        case CTRL_INPUT_L3:     return raw.l3;
        case CTRL_INPUT_R3:     return raw.r3;
        case CTRL_INPUT_MISC:   return raw.misc;
        default: return false;
    }
}

void ControllerMapping::applyMapping(const RawControllerInputs& raw,
                                     bool& motorsToggle, bool& nextEyeColor, bool& prevEyeColor, bool& standUp,
                                     bool& playMusic, bool& subprogram1, bool& subprogram2,
                                     bool& subprogram3, bool& subprogram4, bool& subprogram5,
                                     bool& scan, int& moveAxis, int& turnAxis, int& speedBoostAxis) {
    // Initialize all outputs to false/0
    motorsToggle = false;
    nextEyeColor = false;
    prevEyeColor = false;
    standUp = false;
    playMusic = false;
    subprogram1 = false;
    subprogram2 = false;
    subprogram3 = false;
    subprogram4 = false;
    subprogram5 = false;
    scan = false;
    moveAxis = 0;
    turnAxis = 0;
    speedBoostAxis = 0;
    
    if (!raw.isConnected) {
        return;
    }
    
    // Get mapping for this controller type
    ControllerTypeMapping mapping;
    getMapping(raw.controllerType, mapping);
    
    // Apply button mappings - check each button and see what action it maps to
    for (uint8_t i = 0; i < CTRL_INPUT_BTN_COUNT; i++) {
        uint8_t buttonId = i + 1;  // Button IDs start at 1
        uint8_t action = mapping.buttonActions[i];
        
        if (action == ROBOT_ACTION_NONE) continue;
        
        bool pressed = getButtonValue(raw, buttonId);
        if (!pressed) continue;
        
        // Map to robot action
        switch (action) {
            case ROBOT_ACTION_MOTORS_TOGGLE: motorsToggle = true; break;
            case ROBOT_ACTION_NEXT_EYE_COLOR: nextEyeColor = true; break;
            case ROBOT_ACTION_PREV_EYE_COLOR: prevEyeColor = true; break;
            case ROBOT_ACTION_STAND_UP: standUp = true; break;
            case ROBOT_ACTION_PLAY_MUSIC: playMusic = true; break;
            case ROBOT_ACTION_SUBPROGRAM_1: subprogram1 = true; break;
            case ROBOT_ACTION_SUBPROGRAM_2: subprogram2 = true; break;
            case ROBOT_ACTION_SUBPROGRAM_3: subprogram3 = true; break;
            case ROBOT_ACTION_SUBPROGRAM_4: subprogram4 = true; break;
            case ROBOT_ACTION_SUBPROGRAM_5: subprogram5 = true; break;
            case ROBOT_ACTION_SCAN: scan = true; break;
        }
    }
    
    // Apply axis mappings
    moveAxis = getAxisValue(raw, mapping.moveMapping.sourceAxis);
    if (mapping.moveMapping.inverted) moveAxis = -moveAxis;
    
    turnAxis = getAxisValue(raw, mapping.turnMapping.sourceAxis);
    if (mapping.turnMapping.inverted) turnAxis = -turnAxis;
    
    speedBoostAxis = getAxisValue(raw, mapping.speedBoostMapping.sourceAxis);
    if (mapping.speedBoostMapping.inverted) speedBoostAxis = -speedBoostAxis;
}

