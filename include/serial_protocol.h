#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#include "config.h"

class SerialProtocol {
public:
    // Initialize serial protocol and load name from preferences
    static void init();
    
    // Process incoming serial data - call this in loop()
    static void update();
    
    // Get current robot name
    static const char* getRobotName();
    
    // Set robot name (also saves to preferences)
    static bool setRobotName(const char* name, size_t len);
    
private:
    static char robotName[ROBOT_NAME_MAX_LEN + 1];
    static bool initialized;
    
    // Protocol state machine
    static uint8_t protocolState;
    static uint8_t pendingCommand;
    static uint16_t pendingLength;
    static uint8_t pendingBuffer[SIGN_MSG_MAX_LEN + 4];  // Buffer for commands (sign message is largest)
    static uint16_t pendingBufferIdx;
    
    // Generate default name from crypto ID
    static void generateDefaultName();
    
    // Load name from preferences (returns false if not found)
    static bool loadNameFromPreferences();
    
    // Save name to preferences
    static void saveNameToPreferences();
    
    // Handle received commands
    static void handleGetInfo();
    static void handleSetName(const uint8_t* data, size_t len);
    static void handleGetPublicKey();
    static void handleSignMessage(const uint8_t* data, size_t len);
    
    // Send response
    static void sendResponse(const uint8_t* data, size_t len);
};

#endif // SERIAL_PROTOCOL_H
