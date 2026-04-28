#include "serial_protocol.h"
#include "config.h"
#include "cc1101.h"
#include "ble_server.h"
#include "logger.h"
#include "atecc_crypto.h"
#include <Arduino.h>
#include <Preferences.h>

static const char* MODULE = "Serial";

// Preferences instance for storing robot name
static Preferences prefs;

// Protocol states
enum ProtocolState {
    STATE_IDLE = 0,
    STATE_GOT_START,
    STATE_GOT_CMD,
    STATE_READING_DATA
};

// Static member initialization
char SerialProtocol::robotName[ROBOT_NAME_MAX_LEN + 1] = {0};
bool SerialProtocol::initialized = false;
uint8_t SerialProtocol::protocolState = STATE_IDLE;
uint8_t SerialProtocol::pendingCommand = 0;
uint16_t SerialProtocol::pendingLength = 0;
uint8_t SerialProtocol::pendingBuffer[SIGN_MSG_MAX_LEN + 4] = {0};
uint16_t SerialProtocol::pendingBufferIdx = 0;

void SerialProtocol::init() {
    prefs.begin("robot-serial", false);
    
    // Try to load name from preferences, otherwise generate default
    if (!loadNameFromPreferences()) {
        generateDefaultName();
        saveNameToPreferences();
    }
    
    Logger::infof(MODULE, "Robot name: %s", robotName);
    initialized = true;
}

bool SerialProtocol::loadNameFromPreferences() {
    if (!prefs.isKey("name")) {
        Logger::info(MODULE, "No stored name found in preferences");
        return false;
    }
    
    size_t len = prefs.getString("name", robotName, ROBOT_NAME_MAX_LEN + 1);
    if (len == 0) {
        Logger::info(MODULE, "Stored name is empty");
        return false;
    }
    
    Logger::infof(MODULE, "Loaded name from preferences: %s", robotName);
    return true;
}

void SerialProtocol::saveNameToPreferences() {
    prefs.putString("name", robotName);
    Logger::infof(MODULE, "Saved name to preferences: %s", robotName);
}

void SerialProtocol::generateDefaultName() {
    uint32_t cryptoId = MyCC1101::getCryptoId();
    snprintf(robotName, ROBOT_NAME_MAX_LEN + 1, "%08X", cryptoId);
    Logger::infof(MODULE, "Generated default name: %s (BLE will show as RBM_%s)", robotName, robotName);
}

const char* SerialProtocol::getRobotName() {
    return robotName;
}

static bool isValidNameChar(char c) {
    return (c >= 'a' && c <= 'z') || 
           (c >= 'A' && c <= 'Z') || 
           (c >= '0' && c <= '9');
}

bool SerialProtocol::setRobotName(const char* name, size_t len) {
    if (len < ROBOT_NAME_MIN_LEN) {
        Logger::errorf(MODULE, "Name too short: %d (min %d)", len, ROBOT_NAME_MIN_LEN);
        return false;
    }
    if (len > ROBOT_NAME_MAX_LEN) {
        Logger::errorf(MODULE, "Name too long: %d (max %d)", len, ROBOT_NAME_MAX_LEN);
        return false;
    }
    
    for (size_t i = 0; i < len; i++) {
        if (!isValidNameChar(name[i])) {
            Logger::errorf(MODULE, "Invalid character in name: '%c' (only a-zA-Z0-9 allowed)", name[i]);
            return false;
        }
    }
    
    memcpy(robotName, name, len);
    robotName[len] = '\0';
    
    saveNameToPreferences();
    Logger::infof(MODULE, "Robot name changed to: %s", robotName);
    
    BLEServer::rebuildAdvertising();
    
    return true;
}

void SerialProtocol::update() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        switch (protocolState) {
            case STATE_IDLE:
                if (byte == SERIAL_START_BYTE) {
                    protocolState = STATE_GOT_START;
                }
                break;
                
            case STATE_GOT_START:
                pendingCommand = byte;
                switch (byte) {
                    case SERIAL_CMD_GET_INFO:
                        handleGetInfo();
                        protocolState = STATE_IDLE;
                        break;
                        
                    case SERIAL_CMD_SET_NAME:
                        protocolState = STATE_GOT_CMD;
                        break;
                    
                    case SERIAL_CMD_GET_PUBLIC_KEY:
                        handleGetPublicKey();
                        protocolState = STATE_IDLE;
                        break;
                    
                    case SERIAL_CMD_SIGN_MESSAGE:
                        // Next byte is high byte of message length
                        protocolState = STATE_GOT_CMD;
                        break;
                        
                    default:
                        Logger::warningf(MODULE, "Unknown command: 0x%02X", byte);
                        protocolState = STATE_IDLE;
                        break;
                }
                break;
                
            case STATE_GOT_CMD:
                // Handle SET_NAME command - first byte is length
                if (pendingCommand == SERIAL_CMD_SET_NAME) {
                    pendingLength = byte;
                    if (pendingLength < ROBOT_NAME_MIN_LEN || pendingLength > ROBOT_NAME_MAX_LEN) {
                        uint8_t response[] = {SERIAL_START_BYTE, SERIAL_CMD_SET_NAME, 0x01};
                        sendResponse(response, sizeof(response));
                        protocolState = STATE_IDLE;
                    } else {
                        pendingBufferIdx = 0;
                        protocolState = STATE_READING_DATA;
                    }
                } else if (pendingCommand == SERIAL_CMD_SIGN_MESSAGE) {
                    // First byte is high byte of length, store it and wait for low byte
                    pendingBuffer[0] = byte;  // Store high byte
                    pendingBufferIdx = 1;
                    protocolState = STATE_READING_DATA;
                } else {
                    protocolState = STATE_IDLE;
                }
                break;
                
            case STATE_READING_DATA:
                pendingBuffer[pendingBufferIdx++] = byte;
                
                if (pendingBufferIdx >= sizeof(pendingBuffer)) {
                    Logger::error(MODULE, "Pending buffer overflow");
                    protocolState = STATE_IDLE;
                    break;
                }
                
                // Check if we have enough data for SET_NAME
                if (pendingCommand == SERIAL_CMD_SET_NAME && pendingBufferIdx >= pendingLength) {
                    handleSetName(pendingBuffer, pendingLength);
                    protocolState = STATE_IDLE;
                }
                // Check if we have enough data for SIGN_MESSAGE
                else if (pendingCommand == SERIAL_CMD_SIGN_MESSAGE) {
                    // After receiving the low byte of length (at index 1), calculate total length
                    if (pendingBufferIdx == 2) {
                        // Parse length from first two bytes (big-endian)
                        pendingLength = (pendingBuffer[0] << 8) | pendingBuffer[1];
                        if (pendingLength == 0 || pendingLength > SIGN_MSG_MAX_LEN) {
                            Logger::errorf(MODULE, "Invalid sign message length: %u", pendingLength);
                            uint8_t response[] = {SERIAL_START_BYTE, SERIAL_CMD_SIGN_MESSAGE, 0x01};
                            sendResponse(response, sizeof(response));
                            protocolState = STATE_IDLE;
                        }
                        // Continue reading the message data
                    } else if (pendingBufferIdx >= 2 + pendingLength) {
                        // Have all data: length(2) + message(pendingLength)
                        handleSignMessage(pendingBuffer + 2, pendingLength);
                        protocolState = STATE_IDLE;
                    }
                }
                break;
        }
    }
}

void SerialProtocol::handleGetInfo() {
    Logger::debug(MODULE, "Received GET_INFO command");
    
    const uint16_t firmwareVersion = FIRMWARE_VERSION;
    
    uint8_t nameLen = (uint8_t)strlen(robotName);
    uint8_t responseLen = 5 + nameLen;
    uint8_t response[32];
    
    response[0] = SERIAL_START_BYTE;
    response[1] = SERIAL_CMD_GET_INFO;
    response[2] = (firmwareVersion >> 8) & 0xFF;
    response[3] = firmwareVersion & 0xFF;
    response[4] = nameLen;
    memcpy(&response[5], robotName, nameLen);
    
    sendResponse(response, responseLen);
    Logger::infof(MODULE, "Sent info: version=0x%04X, name=%s", firmwareVersion, robotName);
}

void SerialProtocol::handleSetName(const uint8_t* data, size_t len) {
    Logger::debugf(MODULE, "Received SET_NAME command, len=%d", len);
    
    if (setRobotName((const char*)data, len)) {
        uint8_t response[] = {SERIAL_START_BYTE, SERIAL_CMD_SET_NAME, 0x00};
        sendResponse(response, sizeof(response));
        Logger::info(MODULE, "Name change successful. BLE advertising updated.");
    } else {
        uint8_t response[] = {SERIAL_START_BYTE, SERIAL_CMD_SET_NAME, 0x01};
        sendResponse(response, sizeof(response));
    }
}

void SerialProtocol::sendResponse(const uint8_t* data, size_t len) {
    Serial.write(data, len);
    Serial.flush();
}

void SerialProtocol::handleGetPublicKey() {
    Logger::debug(MODULE, "Received GET_PUBLIC_KEY command");
    
    ATECCCrypto& crypto = ATECCCrypto::getInstance();
    
    // Response: START(1) + CMD(1) + STATUS(1) + KEY(64) = 67 bytes
    uint8_t response[67];
    response[0] = SERIAL_START_BYTE;
    response[1] = SERIAL_CMD_GET_PUBLIC_KEY;
    
    if (!crypto.isInitialized()) {
        Logger::error(MODULE, "Crypto not initialized");
        response[2] = 0x01;  // Error
        sendResponse(response, 3);
        return;
    }
    
    if (!crypto.copyPublicKey(&response[3])) {
        Logger::error(MODULE, "Failed to copy public key");
        response[2] = 0x01;  // Error
        sendResponse(response, 3);
        return;
    }
    
    response[2] = 0x00;  // Success
    sendResponse(response, sizeof(response));
    Logger::info(MODULE, "Sent 64-byte public key");
}

void SerialProtocol::handleSignMessage(const uint8_t* data, size_t len) {
    Logger::debugf(MODULE, "Received SIGN_MESSAGE command, len=%u", len);
    
    ATECCCrypto& crypto = ATECCCrypto::getInstance();
    
    // Response: START(1) + CMD(1) + STATUS(1) + SIGNATURE(64) = 67 bytes
    uint8_t response[67];
    response[0] = SERIAL_START_BYTE;
    response[1] = SERIAL_CMD_SIGN_MESSAGE;
    
    if (!crypto.isInitialized()) {
        Logger::error(MODULE, "Crypto not initialized");
        response[2] = 0x01;  // Error
        sendResponse(response, 3);
        return;
    }
    
    // Sign the message (crypto library will hash it with SHA-256)
    if (!crypto.signRFData(data, len, &response[3])) {
        Logger::error(MODULE, "Failed to sign message");
        response[2] = 0x01;  // Error
        sendResponse(response, 3);
        return;
    }
    
    response[2] = 0x00;  // Success
    sendResponse(response, sizeof(response));
    Logger::infof(MODULE, "Signed %u-byte message, sent 64-byte signature", len);
}
