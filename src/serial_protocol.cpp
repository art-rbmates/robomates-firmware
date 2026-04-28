#include "serial_protocol.h"
#include "config.h"
#include "ble_server.h"
#include "logger.h"
#include <Arduino.h>
#include <Preferences.h>

static const char* MODULE = "Serial";

static Preferences prefs;

enum ProtocolState {
    STATE_IDLE = 0,
    STATE_GOT_START,
    STATE_GOT_CMD,
    STATE_READING_DATA
};

char SerialProtocol::robotName[ROBOT_NAME_MAX_LEN + 1] = {0};
bool SerialProtocol::initialized = false;
uint8_t SerialProtocol::protocolState = STATE_IDLE;
uint8_t SerialProtocol::pendingCommand = 0;
uint16_t SerialProtocol::pendingLength = 0;
uint8_t SerialProtocol::pendingBuffer[64] = {0};
uint16_t SerialProtocol::pendingBufferIdx = 0;

void SerialProtocol::init() {
    prefs.begin("robot-serial", false);

    if (!loadNameFromPreferences()) {
        generateDefaultName();
        saveNameToPreferences();
    }

    Logger::infof(MODULE, "Base name: %s", robotName);
    initialized = true;
}

bool SerialProtocol::loadNameFromPreferences() {
    if (!prefs.isKey("name")) return false;
    String stored = prefs.getString("name", "");
    if (stored.length() < ROBOT_NAME_MIN_LEN || stored.length() > ROBOT_NAME_MAX_LEN) return false;
    strncpy(robotName, stored.c_str(), ROBOT_NAME_MAX_LEN);
    robotName[ROBOT_NAME_MAX_LEN] = '\0';
    return true;
}

void SerialProtocol::saveNameToPreferences() {
    prefs.putString("name", robotName);
}

void SerialProtocol::generateDefaultName() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(robotName, sizeof(robotName), "B%02X%02X%02X", mac[3], mac[4], mac[5]);
}

const char* SerialProtocol::getRobotName() {
    return robotName;
}

bool SerialProtocol::setRobotName(const char* name, size_t len) {
    if (len < ROBOT_NAME_MIN_LEN || len > ROBOT_NAME_MAX_LEN) return false;

    for (size_t i = 0; i < len; i++) {
        char c = name[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-')) {
            return false;
        }
    }

    memcpy(robotName, name, len);
    robotName[len] = '\0';
    saveNameToPreferences();
    BLEServer::rebuildAdvertising();
    Logger::infof(MODULE, "Name changed to: %s", robotName);
    return true;
}

void SerialProtocol::handleGetInfo() {
    uint8_t response[32];
    size_t offset = 0;

    response[offset++] = SERIAL_START_BYTE;
    response[offset++] = SERIAL_CMD_GET_INFO;

    response[offset++] = FIRMWARE_VERSION & 0xFF;
    response[offset++] = (FIRMWARE_VERSION >> 8) & 0xFF;

    uint8_t nameLen = strlen(robotName);
    response[offset++] = nameLen;
    memcpy(&response[offset], robotName, nameLen);
    offset += nameLen;

    sendResponse(response, offset);
}

void SerialProtocol::handleSetName(const uint8_t* data, size_t len) {
    if (len < 1) return;
    uint8_t nameLen = data[0];
    if (len < 1 + nameLen) return;

    bool success = setRobotName((const char*)(data + 1), nameLen);

    uint8_t response[3];
    response[0] = SERIAL_START_BYTE;
    response[1] = SERIAL_CMD_SET_NAME;
    response[2] = success ? 0x00 : 0x01;
    sendResponse(response, sizeof(response));
}

void SerialProtocol::sendResponse(const uint8_t* data, size_t len) {
    Serial.write(data, len);
}

void SerialProtocol::update() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();

        switch (protocolState) {
            case STATE_IDLE:
                if (byte == SERIAL_START_BYTE) protocolState = STATE_GOT_START;
                break;

            case STATE_GOT_START:
                pendingCommand = byte;
                if (pendingCommand == SERIAL_CMD_GET_INFO) {
                    handleGetInfo();
                    protocolState = STATE_IDLE;
                } else if (pendingCommand == SERIAL_CMD_SET_NAME) {
                    protocolState = STATE_GOT_CMD;
                    pendingBufferIdx = 0;
                    pendingLength = 0;
                } else {
                    protocolState = STATE_IDLE;
                }
                break;

            case STATE_GOT_CMD:
                pendingLength = byte;
                if (pendingLength == 0 || pendingLength > sizeof(pendingBuffer)) {
                    protocolState = STATE_IDLE;
                } else {
                    protocolState = STATE_READING_DATA;
                    pendingBufferIdx = 0;
                }
                break;

            case STATE_READING_DATA:
                pendingBuffer[pendingBufferIdx++] = byte;
                if (pendingBufferIdx >= pendingLength) {
                    if (pendingCommand == SERIAL_CMD_SET_NAME) {
                        uint8_t wrapper[64];
                        wrapper[0] = pendingLength;
                        memcpy(&wrapper[1], pendingBuffer, pendingLength);
                        handleSetName(wrapper, 1 + pendingLength);
                    }
                    protocolState = STATE_IDLE;
                }
                break;
        }
    }
}
