#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include "config.h"

class SerialProtocol {
public:
    static void init();
    static void update();
    static const char* getRobotName();
    static bool setRobotName(const char* name, size_t len);

private:
    static char robotName[ROBOT_NAME_MAX_LEN + 1];
    static bool initialized;

    static uint8_t protocolState;
    static uint8_t pendingCommand;
    static uint16_t pendingLength;
    static uint8_t pendingBuffer[64];
    static uint16_t pendingBufferIdx;

    static void generateDefaultName();
    static bool loadNameFromPreferences();
    static void saveNameToPreferences();
    static void handleGetInfo();
    static void handleSetName(const uint8_t* data, size_t len);
    static void sendResponse(const uint8_t* data, size_t len);
};

#endif // SERIAL_PROTOCOL_H
