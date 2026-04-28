#include "central_controller.h"
#include "config.h"
#include "logger.h"
#include "controller_mapping.h"
#include "controller_handler.h"

static const char* MODULE = "CentralCtrl";

// Static member definitions
CentralController::BleControllerInput CentralController::bleInput = {};
bool CentralController::bleInputValid = false;

void CentralController::init() {
    memset(&bleInput, 0, sizeof(bleInput));
    bleInputValid = false;
    Logger::info(MODULE, "Central controller initialized");
}

void CentralController::setBleControllerInput(const BleControllerInput& input) {
    bleInput = input;
    bleInput.timestamp = millis();  // Always use current time
    bleInputValid = true;
    
    Logger::debugf(MODULE, "BLE input: Y=%d, RX=%d, L2=%d, R2=%d, btns=%c%c%c%c",
        input.axisY, input.axisRX, input.l2Axis, input.r2Axis,
        input.a ? 'A' : '-', input.b ? 'B' : '-',
        input.x ? 'X' : '-', input.y ? 'Y' : '-');
}

bool CentralController::isBleControllerActive() {
    if (!bleInputValid) {
        return false;
    }
    
    uint32_t now = millis();
    uint32_t elapsed = now - bleInput.timestamp;
    
    return elapsed < BLE_CONTROLLER_INPUT_TIMEOUT_MS;
}

CentralController::BleControllerInput CentralController::getBleControllerInput() {
    return bleInput;
}

uint32_t CentralController::getLastBleInputTimestamp() {
    return bleInput.timestamp;
}

SharedData::RobotInputs CentralController::bleInputToRobotInputs(const BleControllerInput& input) {
    // Create raw controller inputs structure for mapping
    RawControllerInputs raw = {};
    raw.isConnected = true;
    raw.controllerType = 0;  // Unknown/generic type for BLE controller
    
    // Buttons
    raw.a = input.a;
    raw.b = input.b;
    raw.x = input.x;
    raw.y = input.y;
    raw.l1 = input.l1;
    raw.r1 = input.r1;
    raw.l2Btn = input.l2Axis > 512;  // L2 as button (half pressed)
    raw.r2Btn = input.r2Axis > 512;  // R2 as button (half pressed)
    raw.up = input.up;
    raw.down = input.down;
    raw.left = input.left;
    raw.right = input.right;
    raw.start = false;
    raw.select = false;
    raw.l3 = false;
    raw.r3 = false;
    raw.misc = false;
    
    // Axes
    raw.axisX = input.axisX;
    raw.axisY = input.axisY;
    raw.axisRX = input.axisRX;
    raw.axisRY = input.axisRY;
    raw.brake = input.l2Axis;
    raw.throttle = input.r2Axis;
    raw.timestamp = input.timestamp;
    
    // Apply controller mapping to get robot inputs
    SharedData::RobotInputs robotInputs = {};
    robotInputs.isConnected = true;
    robotInputs.timestamp = millis();
    
    ControllerMapping::applyMapping(raw,
        robotInputs.motorsToggle,
        robotInputs.nextEyeColor,
        robotInputs.prevEyeColor,
        robotInputs.standUp,
        robotInputs.playMusic,
        robotInputs.subprogram1,
        robotInputs.subprogram2,
        robotInputs.subprogram3,
        robotInputs.subprogram4,
        robotInputs.subprogram5,
        robotInputs.scan,
        robotInputs.moveAxis,
        robotInputs.turnAxis,
        robotInputs.speedBoostAxis);
    
    return robotInputs;
}

SharedData::RobotInputs CentralController::getRobotInputs() {
    // Check if BLE controller input is active (within timeout)
    if (isBleControllerActive()) {
        // BLE controller has priority over physical controller
        return bleInputToRobotInputs(bleInput);
    }
    
    // Fall back to physical controller (via SharedData)
    return SharedData::getRobotInputs();
}

