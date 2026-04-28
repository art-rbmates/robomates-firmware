#include "subprogram.h"
#include "config.h"
#include "logger.h"
#include "shared_data.h"
#include "led_controller.h"
#include "motor_hardware.h"
#include "balance_controller.h"
#include <cstring>
#include <SPIFFS.h>

static const char* MODULE = "Subprogram";
static const char* SPIFFS_DIR = "/subprog";

// Static member definitions
Preferences Subprogram::preferences;
SubprogramState Subprogram::state = SubprogramState::IDLE;
CommandType Subprogram::currentCommand = CommandType::NONE;

// Registry
SubprogramEntry Subprogram::registry[SUBPROGRAM_MAX_COUNT];
uint8_t Subprogram::registryCount = 0;

// Button assignments (5 buttons - button 1 maps to default, others unassigned by default)
uint8_t Subprogram::buttonSubprogramIds[5] = {SUBPROGRAM_DEFAULT_ID, 0, 0, 0, 0};

// Currently loaded program
uint8_t* Subprogram::program = nullptr;
uint32_t Subprogram::programLength = 0;
uint32_t Subprogram::programPointer = 0;

CRGB Subprogram::savedEyeColor = CRGB::Black;
bool Subprogram::lastButtonStates[5] = {false, false, false, false, false};

uint32_t Subprogram::commandEndTime = 0;

// Chunked write state
uint8_t Subprogram::writeBufferId = 0;
uint16_t Subprogram::writeBufferLength = 0;
uint8_t Subprogram::writeBuffer[SUBPROGRAM_MAX_PROGRAM_SIZE] = {0};
uint16_t Subprogram::writtenBytes = 0;

// Helper function to get SPIFFS file path for a subprogram
static void getSpiffsPath(uint8_t id, char* path, size_t pathLen) {
    snprintf(path, pathLen, "%s/%u.bin", SPIFFS_DIR, id);
}

// Load bytecode data from SPIFFS
static bool loadDataFromSpiffs(uint8_t id, uint8_t* data, uint16_t& dataLength) {
    char path[32];
    getSpiffsPath(id, path, sizeof(path));
    
    File file = SPIFFS.open(path, "r");
    if (!file) {
        Logger::warningf(MODULE, "SPIFFS: Cannot open %s for reading", path);
        dataLength = 0;
        return false;
    }
    
    size_t fileSize = file.size();
    if (fileSize > SUBPROGRAM_MAX_PROGRAM_SIZE) {
        Logger::errorf(MODULE, "SPIFFS: File %s too large (%u bytes)", path, fileSize);
        file.close();
        dataLength = 0;
        return false;
    }
    
    size_t bytesRead = file.read(data, fileSize);
    file.close();
    
    if (bytesRead != fileSize) {
        Logger::errorf(MODULE, "SPIFFS: Read error on %s (got %u of %u)", path, bytesRead, fileSize);
        dataLength = 0;
        return false;
    }
    
    dataLength = (uint16_t)fileSize;
    Logger::infof(MODULE, "SPIFFS: Loaded %u bytes from %s", dataLength, path);
    return true;
}

// Save bytecode data to SPIFFS
static bool saveDataToSpiffs(uint8_t id, const uint8_t* data, uint16_t dataLength) {
    char path[32];
    getSpiffsPath(id, path, sizeof(path));
    
    File file = SPIFFS.open(path, "w");
    if (!file) {
        Logger::errorf(MODULE, "SPIFFS: Cannot open %s for writing", path);
        return false;
    }
    
    size_t bytesWritten = file.write(data, dataLength);
    file.close();
    
    if (bytesWritten != dataLength) {
        Logger::errorf(MODULE, "SPIFFS: Write error on %s (wrote %u of %u)", path, bytesWritten, dataLength);
        return false;
    }
    
    Logger::infof(MODULE, "SPIFFS: Saved %u bytes to %s", dataLength, path);
    return true;
}

// Delete bytecode data from SPIFFS
static bool deleteDataFromSpiffs(uint8_t id) {
    char path[32];
    getSpiffsPath(id, path, sizeof(path));
    
    if (SPIFFS.exists(path)) {
        if (SPIFFS.remove(path)) {
            Logger::infof(MODULE, "SPIFFS: Deleted %s", path);
            return true;
        } else {
            Logger::errorf(MODULE, "SPIFFS: Failed to delete %s", path);
            return false;
        }
    }
    return true;  // File doesn't exist, consider it deleted
}

// Find the next available ID by scanning existing IDs
// Returns 0 if no ID is available
static uint8_t findNextAvailableId(SubprogramEntry* registry, uint8_t registryCount) {
    // Check IDs from 2 to 255 (ID 1 is reserved for default)
    for (uint8_t candidateId = 2; candidateId != 0; candidateId++) {  // Will wrap at 255->0
        bool idInUse = false;
        for (uint8_t i = 0; i < registryCount; i++) {
            if (registry[i].id == candidateId) {
                idInUse = true;
                break;
            }
        }
        if (!idInUse) {
            return candidateId;
        }
    }
    return 0;  // No available ID (very unlikely with max 8 subprograms)
}

void Subprogram::init() {
    Logger::info(MODULE, "Initializing subprogram module");
    Logger::info(MODULE, "If you see 'SPIFFS: mount failed, -10025' error just wait while the SPIFFS is being formatted. It can take a few seconds.");
    
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {  // true = format if mount fails
        Logger::critical(MODULE, "Failed to mount SPIFFS!");
        return;
    }
    Logger::info(MODULE, "SPIFFS mounted successfully");
    
    // Allocate program buffer for execution
    program = new uint8_t[SUBPROGRAM_MAX_PROGRAM_SIZE];
    if (!program) {
        Logger::critical(MODULE, "Failed to allocate program buffer!");
        return;
    }
    
    // Initialize registry
    memset(registry, 0, sizeof(registry));
    registryCount = 0;
    
    // Create default program (always ID 1)
    createDefaultProgram();
    
    // Load additional subprograms from preferences + SPIFFS
    loadRegistry();
    
    // Load button assignments
    loadButtonAssignments();
    
    Logger::infof(MODULE, "Subprogram module initialized, %u programs loaded", registryCount);
}

void Subprogram::createDefaultProgram() {
    // Create default program entry at index 0
    SubprogramEntry& entry = registry[0];
    entry.id = SUBPROGRAM_DEFAULT_ID;
    strncpy(entry.name, "Default", SUBPROGRAM_NAME_MAX_LEN);
    entry.name[SUBPROGRAM_NAME_MAX_LEN] = '\0';
    entry.isDefault = true;
    
    // Build the bytecode
    uint16_t offset = 0;
    
    auto writeInt16 = [&](int16_t value) {
        memcpy(entry.data + offset, &value, sizeof(int16_t));
        offset += sizeof(int16_t);
    };
    
    auto writeUint32 = [&](uint32_t value) {
        memcpy(entry.data + offset, &value, sizeof(uint32_t));
        offset += sizeof(uint32_t);
    };
    
    auto writeSetAxis = [&](int16_t axisY, int16_t axisRX, uint32_t time) {
        entry.data[offset++] = OPCODE_SET_AXIS;
        writeInt16(axisY);
        writeInt16(axisRX);
        writeUint32(time);
    };

    auto writeWait = [&](uint32_t time) {
        entry.data[offset++] = OPCODE_WAIT;
        writeUint32(time);
    };
    
    // Default program sequence
    writeSetAxis(-300, 0, 500);
    writeSetAxis(0, -300, 500);
    writeWait(500);
    writeSetAxis(0, 500, 75);
    writeSetAxis(0, -500, 75);
    writeSetAxis(0, 500, 75);
    writeSetAxis(0, -500, 75);
    writeSetAxis(0, 500, 75);
    writeSetAxis(0, -500, 75);
    writeSetAxis(0, 500, 1000);
    
    entry.dataLength = offset;
    registryCount = 1;
    
    Logger::infof(MODULE, "Created default program: %u bytes", entry.dataLength);
}

void Subprogram::loadRegistry() {
    preferences.begin(SUBPROGRAM_PREFS_NAMESPACE, true);
    
    // Load number of stored subprograms
    uint8_t storedCount = preferences.getUChar("count", 0);
    
    for (uint8_t i = 0; i < storedCount && registryCount < SUBPROGRAM_MAX_COUNT; i++) {
        char keyId[16], keyName[16], keyLen[16];
        snprintf(keyId, sizeof(keyId), "id%u", i);
        snprintf(keyName, sizeof(keyName), "name%u", i);
        snprintf(keyLen, sizeof(keyLen), "len%u", i);
        
        uint8_t id = preferences.getUChar(keyId, 0);
        if (id == 0 || id == SUBPROGRAM_DEFAULT_ID) continue;  // Skip invalid or default
        
        SubprogramEntry& entry = registry[registryCount];
        entry.id = id;
        entry.isDefault = false;
        
        preferences.getString(keyName, entry.name, SUBPROGRAM_NAME_MAX_LEN + 1);
        entry.dataLength = preferences.getUShort(keyLen, 0);
        
        // Load bytecode data from SPIFFS
        if (entry.dataLength > 0) {
            uint16_t loadedLength = 0;
            if (loadDataFromSpiffs(id, entry.data, loadedLength)) {
                entry.dataLength = loadedLength;
            } else {
                // SPIFFS file missing, set length to 0
                entry.dataLength = 0;
                memset(entry.data, 0, SUBPROGRAM_MAX_PROGRAM_SIZE);
            }
        }
        
        registryCount++;
        Logger::infof(MODULE, "Loaded subprogram ID=%u, name=%s, len=%u", entry.id, entry.name, entry.dataLength);
    }
    
    preferences.end();
}

void Subprogram::saveRegistry() {
    preferences.begin(SUBPROGRAM_PREFS_NAMESPACE, false);
    
    // Count and save non-default entries (metadata only - bytecode goes to SPIFFS)
    uint8_t storedCount = 0;
    for (uint8_t i = 0; i < registryCount; i++) {
        if (!registry[i].isDefault) {
            char keyId[16], keyName[16], keyLen[16];
            snprintf(keyId, sizeof(keyId), "id%u", storedCount);
            snprintf(keyName, sizeof(keyName), "name%u", storedCount);
            snprintf(keyLen, sizeof(keyLen), "len%u", storedCount);
            
            preferences.putUChar(keyId, registry[i].id);
            preferences.putString(keyName, registry[i].name);
            preferences.putUShort(keyLen, registry[i].dataLength);
            
            // Save bytecode data to SPIFFS
            if (registry[i].dataLength > 0) {
                saveDataToSpiffs(registry[i].id, registry[i].data, registry[i].dataLength);
            }
            
            storedCount++;
        }
    }
    
    preferences.putUChar("count", storedCount);
    preferences.end();
    
    Logger::infof(MODULE, "Saved %u subprograms (metadata to Prefs, data to SPIFFS)", storedCount);
}

void Subprogram::loadButtonAssignments() {
    preferences.begin(SUBPROGRAM_PREFS_NAMESPACE, true);
    buttonSubprogramIds[0] = preferences.getUChar("btn1", SUBPROGRAM_DEFAULT_ID);
    buttonSubprogramIds[1] = preferences.getUChar("btn2", 0);
    buttonSubprogramIds[2] = preferences.getUChar("btn3", 0);
    buttonSubprogramIds[3] = preferences.getUChar("btn4", 0);
    buttonSubprogramIds[4] = preferences.getUChar("btn5", 0);
    preferences.end();
    
    Logger::infof(MODULE, "Button assignments: 1=%u, 2=%u, 3=%u, 4=%u, 5=%u", 
        buttonSubprogramIds[0], buttonSubprogramIds[1], buttonSubprogramIds[2],
        buttonSubprogramIds[3], buttonSubprogramIds[4]);
}

void Subprogram::saveButtonAssignments() {
    preferences.begin(SUBPROGRAM_PREFS_NAMESPACE, false);
    preferences.putUChar("btn1", buttonSubprogramIds[0]);
    preferences.putUChar("btn2", buttonSubprogramIds[1]);
    preferences.putUChar("btn3", buttonSubprogramIds[2]);
    preferences.putUChar("btn4", buttonSubprogramIds[3]);
    preferences.putUChar("btn5", buttonSubprogramIds[4]);
    preferences.end();
}

int Subprogram::findEntryIndex(uint8_t id) {
    for (uint8_t i = 0; i < registryCount; i++) {
        if (registry[i].id == id) {
            return i;
        }
    }
    return -1;
}

uint8_t Subprogram::getSubprogramCount() {
    return registryCount;
}

bool Subprogram::getSubprogramInfo(uint8_t index, uint8_t& id, char* name, uint16_t& dataLength) {
    if (index >= registryCount) return false;
    
    id = registry[index].id;
    strncpy(name, registry[index].name, SUBPROGRAM_NAME_MAX_LEN + 1);
    dataLength = registry[index].dataLength;
    return true;
}

bool Subprogram::getSubprogramById(uint8_t id, SubprogramEntry& entry) {
    int idx = findEntryIndex(id);
    if (idx < 0) return false;
    
    memcpy(&entry, &registry[idx], sizeof(SubprogramEntry));
    return true;
}

uint8_t Subprogram::createSubprogram(const char* name) {
    if (registryCount >= SUBPROGRAM_MAX_COUNT) {
        Logger::error(MODULE, "Cannot create subprogram: registry full");
        return 0;
    }
    
    if (strlen(name) == 0 || strlen(name) > SUBPROGRAM_NAME_MAX_LEN) {
        Logger::error(MODULE, "Cannot create subprogram: invalid name length");
        return 0;
    }
    
    // Find the next available ID
    uint8_t newId = findNextAvailableId(registry, registryCount);
    if (newId == 0) {
        Logger::error(MODULE, "Cannot create subprogram: no available ID");
        return 0;
    }
    
    SubprogramEntry& entry = registry[registryCount];
    entry.id = newId;
    strncpy(entry.name, name, SUBPROGRAM_NAME_MAX_LEN);
    entry.name[SUBPROGRAM_NAME_MAX_LEN] = '\0';
    entry.dataLength = 0;
    entry.isDefault = false;
    memset(entry.data, 0, SUBPROGRAM_MAX_PROGRAM_SIZE);
    
    registryCount++;
    saveRegistry();
    
    Logger::infof(MODULE, "Created subprogram ID=%u, name=%s", entry.id, entry.name);
    return entry.id;
}

bool Subprogram::renameSubprogram(uint8_t id, const char* newName) {
    if (id == SUBPROGRAM_DEFAULT_ID) {
        Logger::error(MODULE, "Cannot rename default subprogram");
        return false;
    }
    
    int idx = findEntryIndex(id);
    if (idx < 0) {
        Logger::errorf(MODULE, "Cannot rename: subprogram ID=%u not found", id);
        return false;
    }
    
    if (strlen(newName) == 0 || strlen(newName) > SUBPROGRAM_NAME_MAX_LEN) {
        Logger::error(MODULE, "Cannot rename: invalid name length");
        return false;
    }
    
    strncpy(registry[idx].name, newName, SUBPROGRAM_NAME_MAX_LEN);
    registry[idx].name[SUBPROGRAM_NAME_MAX_LEN] = '\0';
    saveRegistry();
    
    Logger::infof(MODULE, "Renamed subprogram ID=%u to %s", id, newName);
    return true;
}

bool Subprogram::updateSubprogramData(uint8_t id, const uint8_t* data, uint16_t length) {
    if (id == SUBPROGRAM_DEFAULT_ID) {
        Logger::error(MODULE, "Cannot update default subprogram data");
        return false;
    }
    
    int idx = findEntryIndex(id);
    if (idx < 0) {
        Logger::errorf(MODULE, "Cannot update: subprogram ID=%u not found", id);
        return false;
    }
    
    if (length > SUBPROGRAM_MAX_PROGRAM_SIZE) {
        Logger::errorf(MODULE, "Cannot update: data too large (%u > %u)", length, SUBPROGRAM_MAX_PROGRAM_SIZE);
        return false;
    }
    
    memcpy(registry[idx].data, data, length);
    registry[idx].dataLength = length;
    
    // Save data to SPIFFS immediately
    if (length > 0) {
        if (!saveDataToSpiffs(id, data, length)) {
            Logger::errorf(MODULE, "Failed to save subprogram data to SPIFFS");
            return false;
        }
    } else {
        // If length is 0, delete the SPIFFS file
        deleteDataFromSpiffs(id);
    }
    
    // Save metadata to preferences
    saveRegistry();
    
    Logger::infof(MODULE, "Updated subprogram ID=%u data, %u bytes", id, length);
    return true;
}

bool Subprogram::deleteSubprogram(uint8_t id) {
    if (id == SUBPROGRAM_DEFAULT_ID) {
        Logger::error(MODULE, "Cannot delete default subprogram");
        return false;
    }
    
    int idx = findEntryIndex(id);
    if (idx < 0) {
        Logger::errorf(MODULE, "Cannot delete: subprogram ID=%u not found", id);
        return false;
    }
    
    // Delete SPIFFS file first
    deleteDataFromSpiffs(id);
    
    // Clear button assignments if this program was assigned
    for (int i = 0; i < 5; i++) {
        if (buttonSubprogramIds[i] == id) {
            buttonSubprogramIds[i] = 0;
        }
    }
    
    // Shift remaining entries
    for (uint8_t i = idx; i < registryCount - 1; i++) {
        memcpy(&registry[i], &registry[i + 1], sizeof(SubprogramEntry));
    }
    registryCount--;
    
    saveRegistry();
    saveButtonAssignments();
    
    Logger::infof(MODULE, "Deleted subprogram ID=%u", id);
    return true;
}

void Subprogram::setButtonSubprogram(uint8_t buttonNum, uint8_t id) {
    if (buttonNum < 1 || buttonNum > 5) {
        Logger::errorf(MODULE, "Invalid button number: %u (must be 1-5)", buttonNum);
        return;
    }
    
    // Validate ID exists (0 = none is valid)
    if (id != 0 && findEntryIndex(id) < 0) {
        Logger::errorf(MODULE, "Cannot assign button %u: subprogram ID=%u not found", buttonNum, id);
        return;
    }
    
    buttonSubprogramIds[buttonNum - 1] = id;
    saveButtonAssignments();
    Logger::infof(MODULE, "Button %u assigned to subprogram ID=%u", buttonNum, id);
}

uint8_t Subprogram::getButtonSubprogram(uint8_t buttonNum) {
    if (buttonNum < 1 || buttonNum > 5) {
        return 0;
    }
    return buttonSubprogramIds[buttonNum - 1];
}

void Subprogram::getButtonSubprograms(uint8_t* ids) {
    for (int i = 0; i < 5; i++) {
        ids[i] = buttonSubprogramIds[i];
    }
}

int16_t Subprogram::readInt16(uint32_t offset) {
    int16_t value;
    memcpy(&value, program + offset, sizeof(int16_t));
    return value;
}

uint32_t Subprogram::readUint32(uint32_t offset) {
    uint32_t value;
    memcpy(&value, program + offset, sizeof(uint32_t));
    return value;
}

void Subprogram::handleButton(uint8_t buttonNum, bool buttonPressed) {
    if (buttonNum < 1 || buttonNum > 5) {
        return;
    }
    
    uint8_t idx = buttonNum - 1;
    
    if (buttonPressed && !lastButtonStates[idx]) {
        if (state == SubprogramState::IDLE) {
            if (buttonSubprogramIds[idx] != 0) {
                startById(buttonSubprogramIds[idx]);
            }
        } else if (state == SubprogramState::EXECUTING) {
            cancel();
        }
    }
    lastButtonStates[idx] = buttonPressed;
}

bool Subprogram::startById(uint8_t id) {
    int idx = findEntryIndex(id);
    if (idx < 0) {
        Logger::errorf(MODULE, "Cannot start: subprogram ID=%u not found", id);
        return false;
    }
    
    // Load program into execution buffer
    programLength = registry[idx].dataLength;
    if (programLength == 0) {
        Logger::errorf(MODULE, "Cannot start: subprogram ID=%u is empty", id);
        return false;
    }
    
    memcpy(program, registry[idx].data, programLength);
    
    Logger::infof(MODULE, "Starting subprogram ID=%u (%s), %u bytes", id, registry[idx].name, programLength);
    return start();
}

bool Subprogram::start() {
    if (!BalanceController::areMotorsEnabled()) {
        Logger::warning(MODULE, "Cannot start: motors are disabled");
        return false;
    }
    
    if (programLength == 0) {
        Logger::warning(MODULE, "Cannot start: no program loaded");
        return false;
    }
    
    Logger::info(MODULE, "Starting subprogram execution");
    
    saveEyeColor();
    LEDController::setEyes(CRGB::White);
    
    state = SubprogramState::EXECUTING;
    programPointer = 0;
    currentCommand = CommandType::NONE;
    
    executeNextCommand();
    
    return true;
}

void Subprogram::cancel() {
    Logger::info(MODULE, "Subprogram cancelled");
    state = SubprogramState::CANCELLED;
    finishExecution();
}

void Subprogram::finishExecution() {
    stopMotors();
    restoreEyeColor();
    
    SharedData::SubprogramControl control = {};
    control.active = false;
    control.axisY = 0;
    control.axisRX = 0;
    SharedData::setSubprogramControl(control);
    
    state = SubprogramState::IDLE;
    currentCommand = CommandType::NONE;
    
    Logger::info(MODULE, "Subprogram execution finished");
}

void Subprogram::saveEyeColor() {
    savedEyeColor = LEDController::getEyeColor();
}

void Subprogram::restoreEyeColor() {
    LEDController::setEyes(savedEyeColor);
}

bool Subprogram::shouldAbort() {
    if (!BalanceController::areMotorsEnabled()) {
        Logger::warning(MODULE, "Aborting: motors disabled");
        return true;
    }
    
    return false;
}

bool Subprogram::isRunning() {
    return state == SubprogramState::EXECUTING;
}

SubprogramState Subprogram::getState() {
    return state;
}

void Subprogram::update() {
    if (state != SubprogramState::EXECUTING) {
        return;
    }
    
    if (shouldAbort()) {
        finishExecution();
        return;
    }
    
    updateCurrentCommand();
}

void Subprogram::executeNextCommand() {
    if (programPointer >= programLength) {
        Logger::info(MODULE, "Program completed");
        finishExecution();
        return;
    }
    
    uint8_t opcode = program[programPointer];
    programPointer++;
    
    switch (opcode) {
        case OPCODE_SET_AXIS: {
            if (programPointer + 8 > programLength) {
                Logger::error(MODULE, "SET_AXIS: insufficient data");
                finishExecution();
                return;
            }
            
            int16_t axisY = readInt16(programPointer);
            programPointer += sizeof(int16_t);
            int16_t axisRX = readInt16(programPointer);
            programPointer += sizeof(int16_t);
            uint32_t time = readUint32(programPointer);
            programPointer += sizeof(uint32_t);
            
            // if (axisY < -511 || axisY > 511 || axisRX < -511 || axisRX > 511 || time > 60000) {
            //     Logger::errorf(MODULE, "SET_AXIS: invalid params (Y=%d, RX=%d, time=%u)", axisY, axisRX, time);
            //     finishExecution();
            //     return;
            // }
            
            Logger::infof(MODULE, "SET_AXIS: Y=%d, RX=%d, time=%ums", axisY, axisRX, time);
            
            commandEndTime = millis() + time;
            currentCommand = CommandType::SET_AXIS;
            
            SharedData::SubprogramControl control;
            control.active = true;
            control.axisY = axisY;
            control.axisRX = axisRX;
            SharedData::setSubprogramControl(control);
            break;
        }
        
        case OPCODE_WAIT: {
            if (programPointer + 4 > programLength) {
                Logger::error(MODULE, "WAIT: insufficient data");
                finishExecution();
                return;
            }
            
            uint32_t time = readUint32(programPointer);
            programPointer += sizeof(uint32_t);
            
            if (time > 60000) {
                Logger::errorf(MODULE, "WAIT: invalid time=%u", time);
                finishExecution();
                return;
            }
            
            Logger::infof(MODULE, "WAIT: time=%ums", time);
            
            commandEndTime = millis() + time;
            currentCommand = CommandType::WAIT;
            
            stopMotors();
            break;
        }
        
        default:
            Logger::errorf(MODULE, "Unknown opcode: 0x%02X", opcode);
            finishExecution();
            break;
    }
}

void Subprogram::updateCurrentCommand() {
    unsigned long now = millis();
    
    switch (currentCommand) {
        case CommandType::SET_AXIS:
        case CommandType::WAIT: {
            if (now >= commandEndTime) {
                Logger::infof(MODULE, "%s complete", 
                    currentCommand == CommandType::SET_AXIS ? "SET_AXIS" : "WAIT");
                executeNextCommand();
            }
            break;
        }
        
        default:
            break;
    }
}

void Subprogram::stopMotors() {
    SharedData::SubprogramControl control;
    control.active = true;
    control.axisY = 0;
    control.axisRX = 0;
    SharedData::setSubprogramControl(control);
}

// ----- Chunked data transfer methods -----

uint8_t Subprogram::readSubprogramData(uint8_t id, uint16_t offset, uint8_t* buffer, uint8_t maxLen) {
    int idx = findEntryIndex(id);
    if (idx < 0) {
        Logger::errorf(MODULE, "readSubprogramData: ID %u not found", id);
        return 0;
    }
    
    SubprogramEntry& entry = registry[idx];
    
    // For default program, data is in memory
    if (entry.isDefault) {
        if (offset >= entry.dataLength) {
            return 0;
        }
        uint16_t remaining = entry.dataLength - offset;
        uint8_t toRead = (remaining < maxLen) ? remaining : maxLen;
        memcpy(buffer, entry.data + offset, toRead);
        return toRead;
    }
    
    // For user programs, read from SPIFFS
    char path[32];
    getSpiffsPath(id, path, sizeof(path));
    
    File file = SPIFFS.open(path, "r");
    if (!file) {
        Logger::errorf(MODULE, "readSubprogramData: Cannot open %s", path);
        return 0;
    }
    
    if (offset >= file.size()) {
        file.close();
        return 0;
    }
    
    file.seek(offset);
    uint16_t remaining = file.size() - offset;
    uint8_t toRead = (remaining < maxLen) ? remaining : maxLen;
    size_t bytesRead = file.read(buffer, toRead);
    file.close();
    
    return (uint8_t)bytesRead;
}

bool Subprogram::startSubprogramWrite(uint8_t id, uint16_t totalLength) {
    // Check if ID exists and is not default
    int idx = findEntryIndex(id);
    if (idx < 0) {
        Logger::errorf(MODULE, "startSubprogramWrite: ID %u not found", id);
        return false;
    }
    
    if (registry[idx].isDefault) {
        Logger::error(MODULE, "startSubprogramWrite: Cannot write to default program");
        return false;
    }
    
    if (totalLength > SUBPROGRAM_MAX_PROGRAM_SIZE) {
        Logger::errorf(MODULE, "startSubprogramWrite: Length %u exceeds max %u", totalLength, SUBPROGRAM_MAX_PROGRAM_SIZE);
        return false;
    }
    
    // Initialize write buffer
    writeBufferId = id;
    writeBufferLength = totalLength;
    writtenBytes = 0;
    memset(writeBuffer, 0, sizeof(writeBuffer));
    
    Logger::infof(MODULE, "Started write buffer for ID=%u, expecting %u bytes", id, totalLength);
    return true;
}

bool Subprogram::writeSubprogramChunk(uint8_t id, uint16_t offset, const uint8_t* data, uint8_t length) {
    // Verify we're writing to the correct buffer
    if (writeBufferId != id || writeBufferId == 0) {
        Logger::errorf(MODULE, "writeSubprogramChunk: Not writing to ID %u (current=%u)", id, writeBufferId);
        return false;
    }
    
    // Check bounds
    if (offset + length > writeBufferLength) {
        Logger::errorf(MODULE, "writeSubprogramChunk: Offset %u + len %u exceeds expected %u", offset, length, writeBufferLength);
        return false;
    }
    
    // Copy data to buffer
    memcpy(writeBuffer + offset, data, length);
    writtenBytes = max(writtenBytes, (uint16_t)(offset + length));
    
    Logger::debugf(MODULE, "Wrote chunk at offset %u, len %u, total written %u/%u", offset, length, writtenBytes, writeBufferLength);
    return true;
}

bool Subprogram::finishSubprogramWrite(uint8_t id) {
    if (writeBufferId != id || writeBufferId == 0) {
        Logger::errorf(MODULE, "finishSubprogramWrite: Not writing to ID %u", id);
        return false;
    }
    
    // Verify we received all expected data
    if (writtenBytes < writeBufferLength) {
        Logger::warningf(MODULE, "finishSubprogramWrite: Only received %u/%u bytes", writtenBytes, writeBufferLength);
    }
    
    // Save to SPIFFS
    bool success = updateSubprogramData(id, writeBuffer, writeBufferLength);
    
    // Clear write state
    writeBufferId = 0;
    writeBufferLength = 0;
    writtenBytes = 0;
    
    if (success) {
        Logger::infof(MODULE, "Finished writing ID=%u, %u bytes saved", id, writeBufferLength);
    }
    
    return success;
}
