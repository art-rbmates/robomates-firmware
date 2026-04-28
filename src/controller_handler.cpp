#include "controller_handler.h"
#include "controller_mapping.h"
#include "logger.h"
#include "led_controller.h"
#include "ble_server.h"
#include "shared_data.h"
#include "config.h"
#include <Arduino.h>
#include <uni.h>

static const char* MODULE = "Controller";

// External reference to the global Bluepad32 instance
extern Bluepad32 BP32;

// Static member definitions
ControllerPtr ControllerHandler::controller = nullptr;

void ControllerHandler::init() {
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    Logger::info(MODULE, "Bluepad32 initialized");
}

bool ControllerHandler::isConnected() {
    return controller && controller->isConnected();
}

int ControllerHandler::getAxisX() {
    if (!controller || !controller->isConnected()) return 0;
    return applyDeadzone(controller->axisX());
}

int ControllerHandler::getAxisY() {
    if (!controller || !controller->isConnected()) return 0;
    return applyDeadzone(controller->axisY());
}

int ControllerHandler::getAxisRX() {
    if (!controller || !controller->isConnected()) return 0;
    return applyDeadzone(controller->axisRX());
}

int ControllerHandler::getAxisRY() {
    if (!controller || !controller->isConnected()) return 0;
    return applyDeadzone(controller->axisRY());
}

int ControllerHandler::getR2Axis() {
    if (!controller || !controller->isConnected()) return 0;
    return applyDeadzone(controller->throttle());
}

int ControllerHandler::getL2Axis() {
    if (!controller || !controller->isConnected()) return 0;
    return applyDeadzone(controller->brake());
}

bool ControllerHandler::isAButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->a();
}

bool ControllerHandler::isBButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->b();
}

bool ControllerHandler::isXButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->x();
}

bool ControllerHandler::isYButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->y();
}

bool ControllerHandler::isR1ButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->r1();
}

bool ControllerHandler::isL1ButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->l1();
}

bool ControllerHandler::isUpButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->dpad() & DPAD_UP;
}

bool ControllerHandler::isDownButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->dpad() & DPAD_DOWN;
}

bool ControllerHandler::isLeftButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->dpad() & DPAD_LEFT;
}

bool ControllerHandler::isRightButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->dpad() & DPAD_RIGHT;
}

bool ControllerHandler::isL2ButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->l2();
}

bool ControllerHandler::isR2ButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->r2();
}

bool ControllerHandler::isL3ButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->thumbL();
}

bool ControllerHandler::isR3ButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->thumbR();
}

bool ControllerHandler::isStartButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->miscButtons() & 0x02;  // Start/Options
}

bool ControllerHandler::isSelectButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->miscButtons() & 0x04;  // Select/Share
}

bool ControllerHandler::isMiscButtonPressed() {
    if (!controller || !controller->isConnected()) return false;
    return controller->miscButtons() & 0x01;  // Misc/System
}

int ControllerHandler::applyDeadzone(int value) {
    if (abs(value) < CONTROLLER_DEADZONE) {
        return 0;
    }
    return value;
}

uint8_t ControllerHandler::getControllerBattery() {
    if (!controller || !controller->isConnected()) return 0;
    return controller->battery();
}

int ControllerHandler::getControllerModel() {
    if (!controller || !controller->isConnected()) return 0;
    return controller->getModel();
}

void ControllerHandler::onConnectedController(ControllerPtr ctl) {
    Logger::info(MODULE, "Controller connected");
    LEDController::setEyes(CRGB::Green);
    LEDController::showLEDs();
    uni_bt_allowlist_remove_all();
    uni_bt_allowlist_set_enabled(true);
    Logger::info(MODULE, "BLE allowlist enabled because controller connected");
    controller = ctl;
}

void ControllerHandler::onDisconnectedController(ControllerPtr ctl) {
    Logger::info(MODULE, "Controller disconnected");
    uni_bt_allowlist_remove_all();
    uni_bt_allowlist_set_enabled(false);
    controller = nullptr;
}

void ControllerHandler::Rumble() {
    if (!controller || !controller->isConnected()) return;
    controller->playDualRumble(0, 0xFF, 0xFF, 200); // Full rumble for 200ms (delayedStartMs=0, strongMagnitude=0xFF, weakMagnitude=0xFF, durationMs=200)
}

void ControllerHandler::disconnectController() {
    if (controller && controller->isConnected()) {
        Logger::info(MODULE, "Disconnecting controller (BLE priority)");
        controller->disconnect();
        controller = nullptr;
    }
}

void ControllerHandler::update() {
    BP32.update();

    if (!controller || !controller->isConnected()) {
        // No controller connected - inform SharedData
        SharedData::RobotInputs robotInputs = {};
        robotInputs.isConnected = false;
        robotInputs.timestamp = millis();
        SharedData::setRobotInputs(robotInputs);
        return;
    }
    
    // Collect raw controller inputs
    RawControllerInputs raw = {};
    raw.isConnected = true;
    raw.controllerType = getControllerModel();
    
    // Buttons
    raw.a = isAButtonPressed();
    raw.b = isBButtonPressed();
    raw.x = isXButtonPressed();
    raw.y = isYButtonPressed();
    raw.l1 = isL1ButtonPressed();
    raw.r1 = isR1ButtonPressed();
    raw.l2Btn = controller->l2();  // L2 as button
    raw.r2Btn = controller->r2();  // R2 as button
    raw.up = isUpButtonPressed();
    raw.down = isDownButtonPressed();
    raw.left = isLeftButtonPressed();
    raw.right = isRightButtonPressed();
    raw.start = controller->miscButtons() & 0x02;  // Start/Options
    raw.select = controller->miscButtons() & 0x04; // Select/Share
    raw.l3 = controller->thumbL();
    raw.r3 = controller->thumbR();
    raw.misc = controller->miscButtons() & 0x01;   // Misc/System
    
    // Axes
    raw.axisX = getAxisX();
    raw.axisY = getAxisY();
    raw.axisRX = getAxisRX();
    raw.axisRY = getAxisRY();
    raw.brake = getL2Axis();
    raw.throttle = getR2Axis();
    raw.timestamp = millis();
    
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
    
    SharedData::setRobotInputs(robotInputs);
}
