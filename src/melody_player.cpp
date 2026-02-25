#include "melody_player.h"
#include "motor_hardware.h"
#include "logger.h"
#include <SPIFFS.h>
#include <Preferences.h>
#include <cstring>

static const char* MODULE = "Melody";
static Preferences melodyPrefs;

// Default melody (hardcoded)
const Note MelodyPlayer::DEFAULT_MELODY[] = {
    // Motif: G G G Eb—
    {1567.98f, 0.16f}, // G6
    {0.0f,     0.02f}, // breath
    {1567.98f, 0.16f}, // G6
    {0.0f,     0.02f}, // breath
    {1567.98f, 0.16f}, // G6
    {1244.51f, 0.90f}, // Eb6 — LONG

    {0.0f,     0.06f}, // phrase gap

    // Answer: F F F D—
    {1396.91f, 0.16f}, // F6
    {0.0f,     0.02f}, // breath
    {1396.91f, 0.16f}, // F6
    {0.0f,     0.02f}, // breath
    {1396.91f, 0.16f}, // F6
    {1174.66f, 1.10f}  // D6 — LONG
};

const Note MelodyPlayer::STARTUP_MELODY[] = {
    {523.25f, 0.2f},
    {0.0f,     0.2f},
    {1046.50f, 0.2f},
    {0.0f,     0.2f},
    {1046.50f, 0.2f}
};

const size_t MelodyPlayer::STARTUP_MELODY_LENGTH = sizeof(MelodyPlayer::STARTUP_MELODY) / sizeof(MelodyPlayer::STARTUP_MELODY[0]);
const size_t MelodyPlayer::DEFAULT_MELODY_LENGTH = sizeof(MelodyPlayer::DEFAULT_MELODY) / sizeof(MelodyPlayer::DEFAULT_MELODY[0]);

// Static member definitions
Note MelodyPlayer::currentMelody[MELODY_MAX_NOTES];
uint16_t MelodyPlayer::currentMelodyLength = 0;
bool MelodyPlayer::isCustomMelody = false;
float MelodyPlayer::currentAmplitude = MELODY_AMPLITUDE;

// Chunked write state
uint8_t MelodyPlayer::writeBuffer[MELODY_MAX_DATA_SIZE];
uint16_t MelodyPlayer::writeBufferLength = 0;
uint16_t MelodyPlayer::writtenBytes = 0;
bool MelodyPlayer::writeInProgress = false;

void MelodyPlayer::init() {
    Logger::info(MODULE, "Initializing melody storage");
    
    // Load amplitude from preferences
    loadAmplitudeFromPrefs();
    
    // Try to load custom melody from SPIFFS
    if (loadFromSpiffs()) {
        Logger::infof(MODULE, "Loaded custom melody: %u notes", currentMelodyLength);
        isCustomMelody = true;
    } else {
        // No custom melody, use default
        loadDefaultMelody();
        Logger::infof(MODULE, "Using default melody: %u notes", currentMelodyLength);
    }
    
    Logger::infof(MODULE, "Melody amplitude: %.2f", currentAmplitude);
}

void MelodyPlayer::loadDefaultMelody() {
    currentMelodyLength = DEFAULT_MELODY_LENGTH;
    memcpy(currentMelody, DEFAULT_MELODY, DEFAULT_MELODY_LENGTH * sizeof(Note));
    isCustomMelody = false;
}

bool MelodyPlayer::loadFromSpiffs() {
    File file = SPIFFS.open(MELODY_SPIFFS_PATH, "r");
    if (!file) {
        Logger::debug(MODULE, "No custom melody file found");
        return false;
    }
    
    size_t fileSize = file.size();
    if (fileSize == 0 || fileSize > MELODY_MAX_DATA_SIZE || (fileSize % sizeof(Note)) != 0) {
        Logger::warningf(MODULE, "Invalid melody file size: %u bytes", fileSize);
        file.close();
        return false;
    }
    
    uint16_t noteCount = fileSize / sizeof(Note);
    size_t bytesRead = file.read(reinterpret_cast<uint8_t*>(currentMelody), fileSize);
    file.close();
    
    if (bytesRead != fileSize) {
        Logger::errorf(MODULE, "Read error: got %u of %u bytes", bytesRead, fileSize);
        return false;
    }
    
    currentMelodyLength = noteCount;
    Logger::infof(MODULE, "SPIFFS: Loaded %u notes (%u bytes)", noteCount, fileSize);
    return true;
}

bool MelodyPlayer::saveToSpiffs() {
    // Ensure directory exists (SPIFFS is flat, but path with / is allowed)
    File file = SPIFFS.open(MELODY_SPIFFS_PATH, "w");
    if (!file) {
        Logger::error(MODULE, "SPIFFS: Cannot open file for writing");
        return false;
    }
    
    size_t dataSize = currentMelodyLength * sizeof(Note);
    size_t bytesWritten = file.write(reinterpret_cast<const uint8_t*>(currentMelody), dataSize);
    file.close();
    
    if (bytesWritten != dataSize) {
        Logger::errorf(MODULE, "SPIFFS: Write error (wrote %u of %u)", bytesWritten, dataSize);
        return false;
    }
    
    Logger::infof(MODULE, "SPIFFS: Saved %u notes (%u bytes)", currentMelodyLength, dataSize);
    return true;
}

bool MelodyPlayer::deleteFromSpiffs() {
    if (SPIFFS.exists(MELODY_SPIFFS_PATH)) {
        if (SPIFFS.remove(MELODY_SPIFFS_PATH)) {
            Logger::info(MODULE, "SPIFFS: Deleted custom melody");
            return true;
        } else {
            Logger::error(MODULE, "SPIFFS: Failed to delete file");
            return false;
        }
    }
    return true;  // File doesn't exist, consider it deleted
}

uint16_t MelodyPlayer::getMelodyNoteCount() {
    return currentMelodyLength;
}

uint16_t MelodyPlayer::getMelodyDataLength() {
    return currentMelodyLength * sizeof(Note);
}

bool MelodyPlayer::hasCustomMelody() {
    return isCustomMelody;
}

float MelodyPlayer::getAmplitude() {
    return currentAmplitude;
}

float MelodyPlayer::getDefaultAmplitude() {
    return MELODY_AMPLITUDE;
}

void MelodyPlayer::setAmplitude(float amplitude) {
    // Clamp amplitude to reasonable range (0.0 to 2.0)
    if (amplitude < 0.0f) amplitude = 0.0f;
    if (amplitude > 2.0f) amplitude = 2.0f;
    
    currentAmplitude = amplitude;
    saveAmplitudeToPrefs();
    
    Logger::infof(MODULE, "Amplitude set to %.2f", currentAmplitude);
}

void MelodyPlayer::loadAmplitudeFromPrefs() {
    melodyPrefs.begin(MELODY_PREFS_NAMESPACE, true);  // read-only
    currentAmplitude = melodyPrefs.getFloat("amplitude", MELODY_AMPLITUDE);
    melodyPrefs.end();
    
    Logger::debugf(MODULE, "Loaded amplitude from prefs: %.2f", currentAmplitude);
}

void MelodyPlayer::saveAmplitudeToPrefs() {
    melodyPrefs.begin(MELODY_PREFS_NAMESPACE, false);  // read-write
    melodyPrefs.putFloat("amplitude", currentAmplitude);
    melodyPrefs.end();
    
    Logger::debugf(MODULE, "Saved amplitude to prefs: %.2f", currentAmplitude);
}

uint8_t MelodyPlayer::readMelodyData(uint16_t offset, uint8_t* buffer, uint8_t maxLen) {
    uint16_t dataSize = getMelodyDataLength();
    
    if (offset >= dataSize) {
        return 0;
    }
    
    uint16_t remaining = dataSize - offset;
    uint8_t toRead = (remaining < maxLen) ? remaining : maxLen;
    
    memcpy(buffer, reinterpret_cast<const uint8_t*>(currentMelody) + offset, toRead);
    return toRead;
}

bool MelodyPlayer::startMelodyWrite(uint16_t totalLength) {
    if (totalLength == 0) {
        Logger::error(MODULE, "startMelodyWrite: Length cannot be 0");
        return false;
    }
    
    if (totalLength > MELODY_MAX_DATA_SIZE) {
        Logger::errorf(MODULE, "startMelodyWrite: Length %u exceeds max %u", totalLength, MELODY_MAX_DATA_SIZE);
        return false;
    }
    
    if ((totalLength % sizeof(Note)) != 0) {
        Logger::errorf(MODULE, "startMelodyWrite: Length %u not aligned to Note size (%u)", totalLength, sizeof(Note));
        return false;
    }
    
    writeBufferLength = totalLength;
    writtenBytes = 0;
    writeInProgress = true;
    memset(writeBuffer, 0, sizeof(writeBuffer));
    
    Logger::infof(MODULE, "Started melody write, expecting %u bytes (%u notes)", 
                  totalLength, totalLength / sizeof(Note));
    return true;
}

bool MelodyPlayer::writeMelodyChunk(uint16_t offset, const uint8_t* data, uint8_t length) {
    if (!writeInProgress) {
        Logger::error(MODULE, "writeMelodyChunk: No write in progress");
        return false;
    }
    
    if (offset + length > writeBufferLength) {
        Logger::errorf(MODULE, "writeMelodyChunk: Offset %u + len %u exceeds expected %u", 
                       offset, length, writeBufferLength);
        return false;
    }
    
    memcpy(writeBuffer + offset, data, length);
    writtenBytes = max(writtenBytes, (uint16_t)(offset + length));
    
    Logger::debugf(MODULE, "Wrote chunk at offset %u, len %u, total %u/%u", 
                   offset, length, writtenBytes, writeBufferLength);
    return true;
}

bool MelodyPlayer::finishMelodyWrite() {
    if (!writeInProgress) {
        Logger::error(MODULE, "finishMelodyWrite: No write in progress");
        return false;
    }
    
    if (writtenBytes < writeBufferLength) {
        Logger::warningf(MODULE, "finishMelodyWrite: Only received %u/%u bytes", 
                         writtenBytes, writeBufferLength);
    }
    
    // Copy write buffer to current melody
    uint16_t noteCount = writeBufferLength / sizeof(Note);
    memcpy(currentMelody, writeBuffer, writeBufferLength);
    currentMelodyLength = noteCount;
    isCustomMelody = true;
    
    // Save to SPIFFS
    bool success = saveToSpiffs();
    
    // Clear write state
    writeInProgress = false;
    writeBufferLength = 0;
    writtenBytes = 0;
    
    if (success) {
        Logger::infof(MODULE, "Finished melody write: %u notes saved", noteCount);
    } else {
        Logger::error(MODULE, "Failed to save melody to SPIFFS");
    }
    
    return success;
}

bool MelodyPlayer::resetToDefaultMelody() {
    Logger::info(MODULE, "Resetting to default melody");
    
    // Delete custom melody from SPIFFS
    deleteFromSpiffs();
    
    // Load default melody
    loadDefaultMelody();
    
    Logger::infof(MODULE, "Reset complete: using default melody (%u notes)", currentMelodyLength);
    return true;
}

void MelodyPlayer::playTorqueMelodyDuet(
    BLDCMotor& L, BLDCMotor& R,
    const Note* voice1, size_t len1,
    const Note* voice2, size_t len2,
    float amp1, float amp2,
    float supply_v
) {
    auto oldL = L.controller;
    L.controller = MotionControlType::torque;
    
    // Only configure motor2 if we have voice2
    MotionControlType oldR = MotionControlType::torque;
    bool controlMotor2 = (len2 > 0);
    if (controlMotor2) {
        oldR = R.controller;
        R.controller = MotionControlType::torque;
    }

    // State per voice
    size_t i1 = 0, i2 = 0;
    uint32_t t0 = micros();
    uint32_t nStart1 = t0, nStart2 = t0;

    auto hz1 = (len1 ? voice1[0].f : 0.0f);
    auto dur1 = (len1 ? voice1[0].d : 0.0f);
    auto hz2 = (len2 ? voice2[0].f : 0.0f);
    auto dur2 = (len2 ? voice2[0].d : 0.0f);

    uint32_t end1 = nStart1 + (uint32_t)(dur1 * 1e6f);
    uint32_t end2 = nStart2 + (uint32_t)(dur2 * 1e6f);

    uint32_t t = t0;
    uint32_t cycles = 0;

    while (true) {
        t = micros(); 
        cycles++;

        // Advance voice 1 if its note ended
        if (i1 < len1 && (int32_t)(t - end1) >= 0) {
            i1++;
            if (i1 < len1) {
                nStart1 = t;
                hz1 = voice1[i1].f;
                dur1 = voice1[i1].d;
                end1 = nStart1 + (uint32_t)(dur1 * 1e6f);
            } else { 
                hz1 = 0.0f; 
            }
        }

        // Advance voice 2 if its note ended
        if (controlMotor2 && i2 < len2 && (int32_t)(t - end2) >= 0) {
            i2++;
            if (i2 < len2) {
                nStart2 = t;
                hz2 = voice2[i2].f;
                dur2 = voice2[i2].d;
                end2 = nStart2 + (uint32_t)(dur2 * 1e6f);
            } else { 
                hz2 = 0.0f; 
            }
        }

        // Exit when all active voices finished
        bool done1 = (i1 >= len1);
        bool done2 = controlMotor2 ? (i2 >= len2) : true;
        if (done1 && done2) break;

        // Generate samples (Ud=0, pure q-axis)
        float y1 = 0.0f, y2 = 0.0f;
        if (hz1 > 0.0f && i1 < len1) {
            float t1 = 1e-6f * (t - nStart1);
            y1 = amp1 * sinf(2.0f * PI * hz1 * t1);
        }
        if (controlMotor2 && hz2 > 0.0f && i2 < len2) {
            float t2 = 1e-6f * (t - nStart2);
            y2 = amp2 * sinf(2.0f * PI * hz2 * t2);
        }

        // Keep FOC fed; write phase voltages directly (Ud=0, Uq=y)
        MotorHardware::setMotor1PhaseVoltage(supply_v, 0.0f, y1);
        if (controlMotor2) {
            MotorHardware::setMotor2PhaseVoltage(supply_v, 0.0f, y2);
        }
    }

    // Stop + restore
    MotorHardware::setMotor1PhaseVoltage(supply_v, 0.0f, 0.0f);
    L.move(0.0f);
    L.controller = oldL;
    
    if (controlMotor2) {
        MotorHardware::setMotor2PhaseVoltage(supply_v, 0.0f, 0.0f);
        R.move(0.0f);
        R.controller = oldR;
    }

    // Optional: simple telemetry
    float elapsed_s = (micros() - t0) * 1e-6f;
    Logger::infof(MODULE, "time=%.3fs cycles=%lu ~%.0f Hz loop",
                  elapsed_s, (unsigned long)cycles, cycles / elapsed_s);
}

void MelodyPlayer::playMelody(BLDCMotor& L, BLDCMotor& R, float amplitude, float supply_v) {
    Logger::infof(MODULE, "Playing melody (%u notes, custom=%d)", currentMelodyLength, isCustomMelody);
    playTorqueMelodyDuet(L, R,
                         currentMelody, currentMelodyLength,
                         currentMelody, currentMelodyLength,
                         amplitude, amplitude, supply_v);
}

void MelodyPlayer::playStartupMelody(BLDCMotor& L, BLDCMotor& R, float amplitude, float supply_v) {
    Logger::info(MODULE, "Playing startup melody");
    Logger::infof(MODULE, "Startup amplitude: %.2f", amplitude);
    Logger::infof(MODULE, "Startup supply voltage: %.2f", supply_v);
    playTorqueMelodyDuet(L, R,
                         STARTUP_MELODY, STARTUP_MELODY_LENGTH,
                         STARTUP_MELODY, STARTUP_MELODY_LENGTH,
                         amplitude, amplitude, supply_v);
}
