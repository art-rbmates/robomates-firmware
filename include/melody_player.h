#ifndef MELODY_PLAYER_H
#define MELODY_PLAYER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"

struct Note {
    float f;  // Frequency in Hz (0 = rest/silence)
    float d;  // Duration in seconds
};

class MelodyPlayer {
public:
    // Default melodies (hardcoded)
    static const Note DEFAULT_MELODY[];
    static const Note STARTUP_MELODY[];
    static const size_t DEFAULT_MELODY_LENGTH;
    static const size_t STARTUP_MELODY_LENGTH;
    
    // Initialize melody storage (load from SPIFFS or use default)
    static void init();
    
    // Play melodies
    static void playTorqueMelodyDuet(
        BLDCMotor& L, BLDCMotor& R,
        const Note* voice1, size_t len1,
        const Note* voice2, size_t len2,
        float amp1, float amp2,
        float supply_v
    );
    
    static void playMelody(BLDCMotor& L, BLDCMotor& R, float amplitude, float supply_v);
    static void playStartupMelody(BLDCMotor& L, BLDCMotor& R, float amplitude, float supply_v);
    
    // Custom melody management
    static uint16_t getMelodyNoteCount();           // Get number of notes in current melody
    static uint16_t getMelodyDataLength();          // Get total data length in bytes
    static bool hasCustomMelody();                  // Check if using custom melody
    
    // Amplitude management
    static float getAmplitude();                    // Get current amplitude
    static float getDefaultAmplitude();             // Get default amplitude (MELODY_AMPLITUDE)
    static void setAmplitude(float amplitude);      // Set amplitude (persisted to preferences)
    
    // Chunked data transfer
    static uint8_t readMelodyData(uint16_t offset, uint8_t* buffer, uint8_t maxLen);
    static bool startMelodyWrite(uint16_t totalLength);
    static bool writeMelodyChunk(uint16_t offset, const uint8_t* data, uint8_t length);
    static bool finishMelodyWrite();
    static bool resetToDefaultMelody();
    
private:
    // Current melody storage
    static Note currentMelody[MELODY_MAX_NOTES];
    static uint16_t currentMelodyLength;  // Number of notes
    static bool isCustomMelody;
    
    // Amplitude storage
    static float currentAmplitude;
    
    // Chunked write state
    static uint8_t writeBuffer[MELODY_MAX_DATA_SIZE];
    static uint16_t writeBufferLength;
    static uint16_t writtenBytes;
    static bool writeInProgress;
    
    // SPIFFS helpers
    static bool loadFromSpiffs();
    static bool saveToSpiffs();
    static bool deleteFromSpiffs();
    static void loadDefaultMelody();
    
    // Preferences helpers for amplitude
    static void loadAmplitudeFromPrefs();
    static void saveAmplitudeToPrefs();
};

#endif // MELODY_PLAYER_H

