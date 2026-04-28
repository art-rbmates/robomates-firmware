#ifndef SUBPROGRAM_H
#define SUBPROGRAM_H

#include <Arduino.h>
#include <Preferences.h>
#include <FastLED.h>
#include "config.h"

// Bytecode opcodes
#define OPCODE_SET_AXIS     0x01  // SetAxis(axisY: int16_t, axisRX: int16_t, time_ms: uint32_t) - 9 bytes total
#define OPCODE_WAIT         0x02  // Wait(time_ms: uint32_t) - 5 bytes total

// Execution states
enum class SubprogramState {
    IDLE,           // No subprogram running
    EXECUTING,      // Currently executing a command
    PAUSED,         // Paused (not used yet, for future)
    CANCELLED       // Cancelled by user
};

// Current command being executed
enum class CommandType {
    NONE,
    SET_AXIS,
    WAIT
};

// Subprogram entry in the registry
struct SubprogramEntry {
    uint8_t id;                                     // Unique ID (1 = default, 2+ = user created)
    char name[SUBPROGRAM_NAME_MAX_LEN + 1];         // Null-terminated name
    uint16_t dataLength;                            // Length of bytecode data
    uint8_t data[SUBPROGRAM_MAX_PROGRAM_SIZE];      // Bytecode data
    bool isDefault;                                 // True if this is the hardcoded default
};

class Subprogram {
public:
    // Initialize subprogram module and load registry
    static void init();
    
    // Update function - call from main loop
    static void update();
    
    // Start/stop subprogram execution
    static bool startById(uint8_t id);  // Start specific subprogram by ID
    static bool start();                // Start currently loaded subprogram
    static void cancel();
    
    // Check if subprogram is currently running
    static bool isRunning();
    
    // Get current state
    static SubprogramState getState();
    
    // Handle button presses for subprograms 1-5
    static void handleButton(uint8_t buttonNum, bool buttonPressed);
    
    // Registry management
    static uint8_t getSubprogramCount();
    static bool getSubprogramInfo(uint8_t index, uint8_t& id, char* name, uint16_t& dataLength);
    static bool getSubprogramById(uint8_t id, SubprogramEntry& entry);
    static uint8_t createSubprogram(const char* name);  // Returns new ID or 0 on failure
    static bool renameSubprogram(uint8_t id, const char* newName);
    static bool updateSubprogramData(uint8_t id, const uint8_t* data, uint16_t length);
    static bool deleteSubprogram(uint8_t id);
    
    // Button assignment (buttons 1-5)
    static void setButtonSubprogram(uint8_t buttonNum, uint8_t id);  // 0 = none
    static uint8_t getButtonSubprogram(uint8_t buttonNum);
    static void getButtonSubprograms(uint8_t* ids);  // Get all 5 button IDs
    
    // Chunked data transfer
    static uint8_t readSubprogramData(uint8_t id, uint16_t offset, uint8_t* buffer, uint8_t maxLen);
    static bool startSubprogramWrite(uint8_t id, uint16_t totalLength);
    static bool writeSubprogramChunk(uint8_t id, uint16_t offset, const uint8_t* data, uint8_t length);
    static bool finishSubprogramWrite(uint8_t id);
    
private:
    static Preferences preferences;
    static SubprogramState state;
    static CommandType currentCommand;
    
    // Registry storage
    static SubprogramEntry registry[SUBPROGRAM_MAX_COUNT];
    static uint8_t registryCount;
    
    // Button assignments (5 buttons)
    static uint8_t buttonSubprogramIds[5];
    
    // Currently loaded program for execution
    static uint8_t* program;
    static uint32_t programLength;
    static uint32_t programPointer;
    
    // Eye color backup
    static CRGB savedEyeColor;
    
    // Button edge detection (5 buttons)
    static bool lastButtonStates[5];
    
    // Command execution state
    static uint32_t commandEndTime;
    
    // Helper functions
    static void executeNextCommand();
    static void updateCurrentCommand();
    static void finishExecution();
    static void saveEyeColor();
    static void restoreEyeColor();
    
    // Parse bytecode helpers
    static int16_t readInt16(uint32_t offset);
    static uint32_t readUint32(uint32_t offset);
    
    // Create default demo program (ID=1)
    static void createDefaultProgram();
    
    // Check if execution should abort
    static bool shouldAbort();
    
    // Motor control
    static void stopMotors();
    
    // Registry persistence
    static void loadRegistry();
    static void saveRegistry();
    static void saveButtonAssignments();
    static void loadButtonAssignments();
    
    // Find entry by ID
    static int findEntryIndex(uint8_t id);
    
    // Chunked write state
    static uint8_t writeBufferId;  // ID being written to (0 = none)
    static uint16_t writeBufferLength;  // Expected total length
    static uint8_t writeBuffer[SUBPROGRAM_MAX_PROGRAM_SIZE];  // Temp buffer for assembly
    static uint16_t writtenBytes;  // Bytes written so far
};

#endif // SUBPROGRAM_H
