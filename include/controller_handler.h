#ifndef CONTROLLER_HANDLER_H
#define CONTROLLER_HANDLER_H

#include <Bluepad32.h>

class ControllerHandler {
public:
    static void init();
    static void update();
    static bool isConnected();
    
    static int getAxisX();
    static int getAxisY();
    static int getAxisRX();
    static int getAxisRY();
    static bool isAButtonPressed();
    static bool isBButtonPressed();
    static bool isXButtonPressed();
    static bool isYButtonPressed();
    static bool isR1ButtonPressed();
    static bool isL1ButtonPressed();
    static int getR2Axis();
    static int getL2Axis();
    static bool isUpButtonPressed();
    static bool isDownButtonPressed();
    static bool isLeftButtonPressed();
    static bool isRightButtonPressed();
    static bool isL2ButtonPressed();
    static bool isR2ButtonPressed();
    static bool isL3ButtonPressed();
    static bool isR3ButtonPressed();
    static bool isStartButtonPressed();
    static bool isSelectButtonPressed();
    static bool isMiscButtonPressed();
    static uint8_t getControllerBattery();
    static int getControllerModel();

    static void onConnectedController(ControllerPtr ctl);
    static void onDisconnectedController(ControllerPtr ctl);

    static void Rumble();
    
    // Connection management
    static void disconnectController();  // Disconnect current controller
    
private:
    static ControllerPtr controller;
    static int applyDeadzone(int value);
};

#endif // CONTROLLER_HANDLER_H
