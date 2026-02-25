#include "cc1101.h"
#include "logger.h"
#include "led_controller.h"
#include "atecc_crypto.h"
#include "shared_data.h"
#include "ble_server.h"
#include "serial_protocol.h"
#include "adc.h"
#include "temperature_sensor.h"
#include <SPI.h>
#include <RadioLib.h>
#include "scan.h"

static const char* MODULE = "MyCC1101";

// Scan buffer: stores recent scan events to embed in pings
MyCC1101::ScanBufferEntry MyCC1101::scanBuffer[SCAN_SLOTS_IN_PING] = {};
static CRGB savedEyeColorForScan = CRGB::Black;  // Saved eye color during scan indication
static bool scanIndicationActive = false;          // Whether eyes are showing scan indication
static unsigned long scanIndicationStartTime = 0;  // When the last scan indication started

// Initialize CC1101 with correct pins from config.h
// Module(CS, GDO0, GDO2, GDO1)
CC1101 radio = new Module(CS_PIN, GDO0_PIN, RADIOLIB_NC, GDO2_PIN);

// Static member initialization - Connection state
volatile bool MyCC1101::isCentralConnected = false;
volatile bool MyCC1101::isFallDetected = false;
volatile uint8_t MyCC1101::controllerBattery = 0;
volatile int MyCC1101::controllerModel = 0;

// Static member initialization - Triggered ping
unsigned long MyCC1101::triggeredPingTime = 0;

// Static member initialization - Statistics (RX by message type)
volatile uint32_t MyCC1101::statsRxPing = 0;
volatile uint32_t MyCC1101::statsRxUpdateStatus = 0;
volatile uint32_t MyCC1101::statsRxScan = 0;

// Static member initialization - Statistics (TX by message type)
volatile uint32_t MyCC1101::statsTxPing = 0;
volatile uint32_t MyCC1101::statsTxUpdateStatus = 0;
volatile uint32_t MyCC1101::statsTxScan = 0;

// Static member initialization - Statistics (Errors)
volatile uint32_t MyCC1101::statsErrCrc = 0;
volatile uint32_t MyCC1101::statsErrValidation = 0;
volatile uint32_t MyCC1101::statsErrZeroLength = 0;
volatile uint32_t MyCC1101::statsErrReadFailed = 0;
volatile uint32_t MyCC1101::statsErrTxFailed = 0;
volatile uint32_t MyCC1101::statsErrTxTimeout = 0;

// Static member initialization - Central update queue (SPSC ring buffer)
MyCC1101::CentralUpdateQueueItem MyCC1101::centralUpdateQueue[CENTRAL_UPDATE_QUEUE_SIZE] = {};
volatile uint8_t MyCC1101::centralUpdateQueueHead = 0;
volatile uint8_t MyCC1101::centralUpdateQueueTail = 0;

// Static member initialization - RF TX queue
MyCC1101::RfTxQueueItem MyCC1101::rfTxQueue[RF_TX_QUEUE_SIZE] = {};
volatile uint8_t MyCC1101::rfTxQueueHead = 0;
volatile uint8_t MyCC1101::rfTxQueueTail = 0;
volatile uint8_t MyCC1101::rfTxQueueCount = 0;

// Static member initialization - Crypto ID
uint32_t MyCC1101::cryptoId = 0;

// Static member initialization - Pending LED update buffer
MyCC1101::PendingLedEntry MyCC1101::pendingLeds[UPDATE_STATUS_NUM_LEDS] = {};
volatile uint8_t MyCC1101::pendingLedCount = 0;
volatile bool MyCC1101::pendingEyeColorSet = false;
uint8_t MyCC1101::pendingEyeR = 0;
uint8_t MyCC1101::pendingEyeG = 0;
uint8_t MyCC1101::pendingEyeB = 0;

// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;
volatile bool receivedFlag = false;

// Track if we actually started a transmission (to avoid calling finishTransmit on spurious interrupts)
static volatile bool isTransmitting = false;
static unsigned long transmitStartTime = 0;
static const unsigned long TX_TIMEOUT_MS = 500;  // Max time to wait for TX complete interrupt

// this function is called when a complete packet
// is transmitted by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setTransmittedFlag(void) {
  // we sent a packet, set the flag
  transmittedFlag = true;
}

#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setReceivedFlag(void) {
  // we received a packet, set the flag
  receivedFlag = true;
}


void MyCC1101::init() {
    
    if (DISABLE_CC1101) {
        return;
    }

    Logger::info(MODULE, "Initializing CC1101 with RadioLib...");
    
    // Verify message struct sizes at compile time
    static_assert(sizeof(PingMessage) == MSG_SIZE_PING, "PingMessage size mismatch!");
    // UpdateStatusMessage is now variable length (18-243 bytes), no static size check
    static_assert(sizeof(UpdateStatusRobotEntry) == MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY, "UpdateStatusRobotEntry size mismatch!");
    static_assert(sizeof(ScanMessage) == MSG_SIZE_SCAN, "ScanMessage size mismatch!");
    
    cryptoId = getCryptoId();
    Logger::infof(MODULE, "Crypto ID: 0x%08X", cryptoId);

    // Initialize SPI with correct pins from config.h
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

    Logger::infof(MODULE, "Initializing CC1101 at %f MHz...", CC1101_FREQUENCY);
    
    // Initialize CC1101 at 868.3 MHz
    int state = radio.begin(
        CC1101_FREQUENCY,   // freq MHz
        CC1101_BIT_RATE,     // bit rate kbps
        CC1101_FREQUENCY_DEVIATION,    // freq deviation kHz
        CC1101_RX_BANDWIDTH,    // RX bandwidth kHz
        CC1101_TX_POWER,      // TX power dBm (CC1101 practical max ~10–12 dBm)
        CC1101_PREAMBLE_LENGTH       // preamble length in bits
      );

    radio.setPacketSentAction(setTransmittedFlag);
    radio.setPacketReceivedAction(setReceivedFlag);

    state = radio.setNodeAddress(0x05, 2);
    if (state == RADIOLIB_ERR_NONE) {
        Logger::info(MODULE, "Node address set to 0x05");
    } else {
        Logger::errorf(MODULE, "Failed to set node address, code %d", state);
    }

    // Configure CCA (Clear Channel Assessment) for TX-if-CCA
    // 1) Use "RSSI below threshold AND not receiving packet" CCA mode
    state = radio.setCcaMode(RADIOLIB_CC1101_CCA_MODE_RSSI_THR_RX_PKT);
    if (state == RADIOLIB_ERR_NONE) {
        Logger::info(MODULE, "CCA mode set to RSSI_THR_RX_PKT");
    } else {
        Logger::errorf(MODULE, "Failed to set CCA mode, code %d", state);
    }

    // 2) Set carrier sense threshold: +2 dB above MAGN_TARGET to avoid "always busy" in noisy rooms
    // absThrDb is -8..+7 relative to MAGN_TARGET. -8 disables ABS threshold.
    state = radio.setCarrierSenseThreshold(+2, RADIOLIB_CC1101_CARRIER_SENSE_REL_THR_OFF);
    if (state == RADIOLIB_ERR_NONE) {
        Logger::info(MODULE, "Carrier sense threshold set to +2dB");
    } else {
        Logger::errorf(MODULE, "Failed to set carrier sense threshold, code %d", state);
    }

    // 3) Enable TX-if-CCA behavior in startTransmit() with bounded retries
    radio.enableTxIfCca(true, /*maxRetries=*/6, /*baseBackoffMs=*/15);
    Logger::info(MODULE, "TX-if-CCA enabled (maxRetries=6, baseBackoffMs=15)");

    Serial.print(F("[CC1101] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Logger::info(MODULE, "Starting to listen ... success!");
    } else {
        Logger::errorf(MODULE, "Failed to start receive, code %d", state);
    }

    // Clear any spurious interrupt flags that may have been set during initialization
    transmittedFlag = false;
    receivedFlag = false;
    isTransmitting = false;

    Logger::info(MODULE, "CC1101 initialization complete - ready for communication");
}

uint8_t MyCC1101::batteryMvToEncoded(uint16_t millivolts) {
    // Encode battery voltage into 1 byte: 0=2500mV, 255=4200mV
    // Range: 1700mV, Resolution: ~6.67mV per step
    
    if (millivolts >= BATTERY_ENCODE_MAX_MV) {
        return 255;
    } else if (millivolts <= BATTERY_ENCODE_MIN_MV) {
        return 0;
    } else {
        return ((millivolts - BATTERY_ENCODE_MIN_MV) * 255) / (BATTERY_ENCODE_MAX_MV - BATTERY_ENCODE_MIN_MV);
    }
}

size_t MyCC1101::getExpectedMessageSize(uint8_t dataType) {
    switch (dataType) {
        case DATA_TYPE_PING:
            return MSG_SIZE_PING;
        case DATA_TYPE_UPDATE_STATUS:
            return 0;  // Variable length: 18-243 bytes, validated separately
        case DATA_TYPE_SCAN:
            return MSG_SIZE_SCAN;
        default:
            return 0;
    }
}

// Helper function to validate UPDATE_STATUS message length
static bool isValidUpdateStatusLength(size_t msgLen) {
    // Must be at least MSG_SIZE_UPDATE_STATUS_MIN (18 bytes)
    if (msgLen < MSG_SIZE_UPDATE_STATUS_MIN) return false;
    // Must not exceed MSG_SIZE_UPDATE_STATUS_MAX (243 bytes for 16 robots)
    if (msgLen > MSG_SIZE_UPDATE_STATUS_MAX) return false;
    // Length must be: 1 (data_type) + N * 15 (entries) + 2 (checksum)
    // So: (msgLen - 3) must be divisible by 15
    return ((msgLen - 3) % MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY) == 0;
}

uint8_t MyCC1101::isValidData(const uint8_t* data, size_t length) {
    // Minimum size: canary_start(1) + data_type(1) + min_payload + checksum(2) + canary_end(1)
    if (length < 6) {  // Absolute minimum
        return DATA_STRUCTURE_VALIDATION_INCORRECT_LENGTH;
    }
    if (data[0] != DATA_STRUCTURE_CANARY_START) {
        return DATA_STRUCTURE_VALIDATION_INCORRECT_START_CANARY;
    }
    if (data[length - 1] != DATA_STRUCTURE_CANARY_END) {
        return DATA_STRUCTURE_VALIDATION_INCORRECT_END_CANARY;
    }
    
    // Check data type and expected size
    uint8_t dataType = data[1]; // data_type is right after canary_start
    size_t msgLen = length - 2;  // Message length without canaries
    
    // Handle UPDATE_STATUS separately (variable length)
    if (dataType == DATA_TYPE_UPDATE_STATUS) {
        if (!isValidUpdateStatusLength(msgLen)) {
            return DATA_STRUCTURE_VALIDATION_INCORRECT_LENGTH;
        }
        return DATA_STRUCTURE_VALIDATION_VALID;
    }
    
    // For fixed-size messages
    size_t expectedSize = getExpectedMessageSize(dataType);
    if (expectedSize == 0) {
        return DATA_STRUCTURE_VALIDATION_UNKNOWN_TYPE;
    }
    
    // Total length should be expectedSize + 2 canaries
    if (length != expectedSize + 2) {
        return DATA_STRUCTURE_VALIDATION_INCORRECT_LENGTH;
    }
    
    return DATA_STRUCTURE_VALIDATION_VALID;
}

void MyCC1101::update() {

    if (DISABLE_CC1101) {
        return;
    }

    sendPing();
    sendScan();

    // Process UPDATE_STATUS queue from central (one per update cycle)
    processCentralUpdateQueue();
    
    // Process RF TX queue (one per update cycle to avoid blocking)
    processRfTxQueue();

    if (receivedFlag) {
        receivedFlag = false;
        size_t packetLength = radio.getPacketLength();
        if (packetLength > 0) {
            packetLength--; // because of address filtering, we need to subtract 1, check MYFIX in CC1101.cpp
            // Fixed-size buffer instead of VLA to prevent stack overflow from corrupt packet lengths
            uint8_t data[MSG_SIZE_MAX + 2];
            if (packetLength > sizeof(data)) {
                Logger::errorf(MODULE, "Packet too large (%d bytes), dropping", packetLength);
                statsErrValidation++;
                radio.startReceive();
            } else {
                int state = radio.readData(data, 0);
                int state_start_receive = radio.startReceive();
                if (state_start_receive != RADIOLIB_ERR_NONE) {
                    Logger::errorf(MODULE, "Failed to start receive, code %d", state_start_receive);
                }
                if (state == RADIOLIB_ERR_NONE) {
                    uint8_t validationState = isValidData(data, packetLength);
                    if (validationState == DATA_STRUCTURE_VALIDATION_VALID) {
                        // Extract message without canaries
                        size_t msgLen = packetLength - 2;
                        handleRxPayload(data + 1, msgLen, radio.getRSSI());
                        // RX stats are incremented per message type in handleRxPayload
                    } else {
                        Logger::errorf(MODULE, "Invalid data, validation state: %d", validationState);
                        statsErrValidation++;
                    }
                } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
                    Logger::error(MODULE, "CRC error!");
                    statsErrCrc++;
                } else {
                    Logger::errorf(MODULE, "Failed to read data, code %d", state);
                    statsErrReadFailed++;
                }
            }
        } else {
            Logger::error(MODULE, "Packet length is 0");
            statsErrZeroLength++;
            int state_start_receive = radio.startReceive();
            if (state_start_receive != RADIOLIB_ERR_NONE) {
                Logger::errorf(MODULE, "Failed to start receive, code %d", state_start_receive);
            }
        }        
    }

    if(transmittedFlag) {
        transmittedFlag = false;
        
        // Only process if we actually started a transmission
        // This avoids errors from spurious interrupts during startup or idle periods
        if (isTransmitting) {
            isTransmitting = false;
            
            // Note: We skip finishTransmit() and go directly to receive mode.
            // finishTransmit() can timeout (-5) on first transmissions after boot,
            // even though the transmission actually completed (interrupt fired).
            // The interrupt callback indicates TX is done, so we just switch to RX.
            Logger::debug(MODULE, "Finished transmitting");
            int state_start_receive = radio.startReceive();
            if (state_start_receive != RADIOLIB_ERR_NONE) {
                Logger::errorf(MODULE, "Failed to start receive, code %d", state_start_receive);
            }
        } else {
            Logger::debug(MODULE, "Spurious TX interrupt ignored (not transmitting)");
        }
    }
}

void MyCC1101::handleRxPayload(const uint8_t* data, size_t length, int8_t rssi) {
    if (length < sizeof(MessageHeader)) {
        Logger::error(MODULE, "Message too short for header");
        return;
    }
    
    uint8_t dataType = data[0];
    Logger::debugf(MODULE, "Received message type 0x%02X, length %d, RSSI: %d", dataType, length, rssi);
    
    switch (dataType) {
        case DATA_TYPE_PING:
            if (length == MSG_SIZE_PING) {
                PingMessage msg;
                memcpy(&msg, data, sizeof(msg));
                handlePing(msg, rssi);
                statsRxPing++;
            }
            break;
            
        case DATA_TYPE_UPDATE_STATUS:
            if (isValidUpdateStatusLength(length)) {
                // handleUpdateStatus processes the multi-robot format
                // Robots receiving via RF only apply their own data, don't relay
                handleUpdateStatus(data, length, rssi);
                statsRxUpdateStatus++;
            }
            break;
            
        case DATA_TYPE_SCAN:
            if (length == MSG_SIZE_SCAN) {
                ScanMessage msg;
                memcpy(&msg, data, sizeof(msg));
                handleScan(msg, rssi);
                statsRxScan++;
            }
            break;
            
        default:
            Logger::errorf(MODULE, "Unknown message type: 0x%02X", dataType);
            statsErrValidation++;  // Unknown message type is a validation error
            break;
    }
}

void MyCC1101::handlePing(const PingMessage& msg, int8_t rssi) {
    Logger::debugf(MODULE, "PING from crypto_id=0x%08X, battery_enc=%d, controller_type=%d, controller_battery=%d%%",
                  msg.crypto_id, msg.robot_battery, msg.controller_type, msg.controller_battery);
    
    // Forward to central via BLE if connected (mark as relayed)
    if (getIsCentralConnected()) {
        PingMessage relayedMsg = msg;
        relayedMsg.is_relayed = 1;  // Mark as relayed from RF
        relayedMsg.checksum = calculateChecksum(relayedMsg);
        BLEServer::sendPingToCentral(relayedMsg);
    }
}

bool MyCC1101::handleUpdateStatus(const uint8_t* data, size_t len, int8_t rssi) {
    // Multi-robot UPDATE_STATUS format:
    // data_type(1) + N * (crypto_id(4) + leds(10) + speed_torque(1)) + checksum(2)
    // Returns true if this robot's data was found and applied
    
    uint8_t robotCount = getUpdateStatusRobotCount(len);
    Logger::debugf(MODULE, "UPDATE_STATUS with %d robot(s)", robotCount);
    
    // Find our entry in the message
    const uint8_t* entryPtr = data + 1;  // Skip data_type byte
    bool foundOurs = false;
    
    for (uint8_t i = 0; i < robotCount; i++) {
        // Parse entry: crypto_id(4) + leds(10) + speed_torque(1)
        uint32_t entryCryptoId = 
            (uint32_t)entryPtr[0] | 
            ((uint32_t)entryPtr[1] << 8) | 
            ((uint32_t)entryPtr[2] << 16) | 
            ((uint32_t)entryPtr[3] << 24);
        
        Logger::debugf(MODULE, "Entry %d: crypto_id=0x%08X", i, entryCryptoId);
        
        // Check if this entry is for us (RF_BROADCAST_ADDR not supported in multi-robot format)
        if (entryCryptoId == cryptoId) {
            foundOurs = true;
            Logger::info(MODULE, "Applying UPDATE_STATUS for our robot");
            
            // Parse LEDs (10 bytes for indices 2-11)
            // Format: first bit = 0 means LED off, first bit = 1 means LED on
            // When LED on: lower 7 bits = R*25 + G*5 + B (0-124)
            // R,G,B range 0-4 maps to: 0, 63, 127, 191, 255
            static const uint8_t colorLookup[5] = {0, 63, 127, 191, 255};
            
            const uint8_t* ledsPtr = entryPtr + 4;
            uint8_t count = 0;
            for (int ledIdx = 0; ledIdx < UPDATE_STATUS_NUM_LEDS; ledIdx++) {
                uint8_t encoded = ledsPtr[ledIdx];
                
                // First bit = 0 means LED off (skip, don't change)
                if ((encoded & 0x80) == 0) {
                    continue;
                }
                
                // LED is on, decode color from lower 7 bits
                uint8_t colorVal = encoded & 0x7F;
                
                // Decode: colorVal = R*25 + G*5 + B, where R,G,B are 0-4
                uint8_t r5 = colorVal / 25;
                uint8_t rem = colorVal % 25;
                uint8_t g5 = rem / 5;
                uint8_t b5 = rem % 5;
                
                // Map 0-4 to actual RGB values: 0, 63, 127, 191, 255
                uint8_t r = colorLookup[r5];
                uint8_t g = colorLookup[g5];
                uint8_t b = colorLookup[b5];
                
                // LED index in the actual array: ledIdx + 2 (skip eyes at 0,1)
                int actualLedIdx = ledIdx + 2;
                
                Logger::debugf(MODULE, "Setting LED %d to RGB(%d, %d, %d)", actualLedIdx, r, g, b);
                
                // Buffer LED changes instead of writing directly (safe for cross-core access)
                if (count < UPDATE_STATUS_NUM_LEDS) {
                    pendingLeds[count].ledIndex = actualLedIdx;
                    pendingLeds[count].r = r;
                    pendingLeds[count].g = g;
                    pendingLeds[count].b = b;
                    count++;
                }
            }
            // Memory barrier: ensure all LED data is written before count becomes visible
            __sync_synchronize();
            pendingLedCount = count;
            
            // Parse speed/torque byte (upper 4 bits = speed, lower 4 bits = torque)
            uint8_t speedTorqueByte = entryPtr[14];
            uint8_t speedNibble = (speedTorqueByte >> 4) & 0x0F;  // 0-15
            uint8_t torqueNibble = speedTorqueByte & 0x0F;        // 0-15
            
            // Convert 0-15 to 0.0-1.0
            float speedCoeff = speedNibble / 15.0f;
            float torqueCoeff = torqueNibble / 15.0f;
            
            SharedData::SystemState system = SharedData::getSystemState();
            system.speedCoefficient = speedCoeff;
            system.torqueCoefficient = torqueCoeff;
            SharedData::setSystemState(system);
            Logger::infof(MODULE, "Speed coefficient: %.3f, Torque coefficient: %.3f", 
                          speedCoeff, torqueCoeff);
            
            // Schedule a triggered ping after TRIGGERED_PING_DELAY_MS
            triggeredPingTime = millis();
            
            // Found and applied our data, no need to continue (crypto_ids are unique)
            break;
        }
        
        // Move to next entry
        entryPtr += MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY;
    }
    
    if (!foundOurs) {
        Logger::debug(MODULE, "UPDATE_STATUS: no entry for our crypto_id");
    }
    
    return foundOurs;
}

void MyCC1101::handleScan(const ScanMessage& msg, int8_t rssi) {
    Logger::debugf(MODULE, "SCAN from crypto_id=0x%08X, RSSI=%d", msg.crypto_id, rssi);
    
    // Don't respond to our own scans
    if (msg.crypto_id == cryptoId) {
        return;
    }
    Logger::infof(MODULE, "SCAN RSSI %d, threshold %d", rssi, RSSI_SCAN_THRESHOLD);
    if (rssi >= RSSI_SCAN_THRESHOLD) {
        Logger::infof(MODULE, "SCAN detected from 0x%08X (RSSI %d >= %d)", msg.crypto_id, rssi, RSSI_SCAN_THRESHOLD);
        
        // Record scan event in buffer (dedup + store for ping embedding)
        recordScanEvent(msg.crypto_id);
        
        // Visual indication: set eyes to magenta briefly (buffered for Core 1 to apply)
        if (!scanIndicationActive) {
            savedEyeColorForScan = LEDController::getEyeColor();
        }
        scanIndicationActive = true;
        scanIndicationStartTime = millis();
        pendingEyeR = 255; pendingEyeG = 0; pendingEyeB = 255;  // Magenta
        __sync_synchronize();
        pendingEyeColorSet = true;
    } else {
        Logger::debugf(MODULE, "SCAN RSSI %d < threshold %d, ignoring", rssi, RSSI_SCAN_THRESHOLD);
    }
}

void MyCC1101::applyPendingLedUpdate() {
    // Called from Core 1 (main loop) to apply LED changes buffered by Core 0
    
    // Apply pending body LEDs (from UPDATE_STATUS)
    uint8_t count = pendingLedCount;
    if (count > 0) {
        // Memory barrier: ensure we see the latest LED data written by Core 0
        __sync_synchronize();
        
        for (uint8_t i = 0; i < count && i < UPDATE_STATUS_NUM_LEDS; i++) {
            LEDController::setLED(static_cast<LEDPositions>(pendingLeds[i].ledIndex), 
                                  CRGB(pendingLeds[i].r, pendingLeds[i].g, pendingLeds[i].b));
        }
        
        // Clear pending flag
        pendingLedCount = 0;
    }
    
    // Apply pending eye color (from scan indication)
    if (pendingEyeColorSet) {
        __sync_synchronize();
        LEDController::setEyes(CRGB(pendingEyeR, pendingEyeG, pendingEyeB));
        pendingEyeColorSet = false;
    }
}

void MyCC1101::setIsCentralConnected(bool connected) {
    isCentralConnected = connected;
}

void MyCC1101::setIsFallDetected(bool fallDetected) {
    isFallDetected = fallDetected;
}

void MyCC1101::setControllerBattery(uint8_t battery) {
    controllerBattery = battery;
}

void MyCC1101::setControllerModel(int model) {
    controllerModel = model;
}

MyCC1101::Stats MyCC1101::getAndResetStats() {
    Stats stats;
    
    // RX by message type
    stats.rx_ping = statsRxPing;
    stats.rx_update_status = statsRxUpdateStatus;
    stats.rx_scan = statsRxScan;
    
    // TX by message type
    stats.tx_ping = statsTxPing;
    stats.tx_update_status = statsTxUpdateStatus;
    stats.tx_scan = statsTxScan;
    
    // Errors
    stats.err_crc = statsErrCrc;
    stats.err_validation = statsErrValidation;
    stats.err_zero_length = statsErrZeroLength;
    stats.err_read_failed = statsErrReadFailed;
    stats.err_tx_failed = statsErrTxFailed;
    stats.err_tx_timeout = statsErrTxTimeout;
    
    // Reset all counters
    statsRxPing = 0;
    statsRxUpdateStatus = 0;
    statsRxScan = 0;
    statsTxPing = 0;
    statsTxUpdateStatus = 0;
    statsTxScan = 0;
    statsErrCrc = 0;
    statsErrValidation = 0;
    statsErrZeroLength = 0;
    statsErrReadFailed = 0;
    statsErrTxFailed = 0;
    statsErrTxTimeout = 0;
    
    return stats;
}

// Maximum robots per RF packet to stay within CC1101's 64-byte FIFO
// Packet = canary(1) + data_type(1) + N*entry(15) + checksum(2) + canary(1) + address(1)
// 64 >= 1 + 1 + N*15 + 2 + 1 + 1 => N <= 3.86 => max 3 robots per RF packet
#define RF_MAX_ROBOTS_PER_PACKET 3

void MyCC1101::setUpdateStatusFromCentral(const uint8_t* data, size_t len) {
    // Multi-robot UPDATE_STATUS format received from BLE
    // data_type(1) + N * (crypto_id(4) + leds(10) + speed_torque(1)) + checksum(2)
    
    if (!isValidUpdateStatusLength(len)) {
        Logger::errorf(MODULE, "Invalid UPDATE_STATUS length: %d", len);
        return;
    }
    
    uint8_t robotCount = getUpdateStatusRobotCount(len);
    Logger::infof(MODULE, "UPDATE_STATUS from central with %d robot(s)", robotCount);
    
    // First, apply our own data immediately (don't wait for queue processing)
    handleUpdateStatus(data, len, 0);
    
    // Now build message(s) to forward via RF:
    // - Skip our own crypto_id
    // - Skip duplicate crypto_ids (keep only first occurrence)
    // - Split into multiple packets if needed (CC1101 FIFO is 64 bytes)
    
    // First pass: collect all entries to forward (excluding ours and duplicates)
    struct ForwardEntry {
        const uint8_t* data;  // Pointer to entry in original message
    };
    ForwardEntry entriesToForward[UPDATE_STATUS_MAX_ROBOTS];
    uint8_t forwardCount = 0;
    
    // Track seen crypto_ids to filter duplicates
    uint32_t seenCryptoIds[UPDATE_STATUS_MAX_ROBOTS];
    uint8_t seenCount = 0;
    
    const uint8_t* entryPtr = data + 1;
    for (uint8_t i = 0; i < robotCount; i++) {
        uint32_t entryCryptoId = 
            (uint32_t)entryPtr[0] | 
            ((uint32_t)entryPtr[1] << 8) | 
            ((uint32_t)entryPtr[2] << 16) | 
            ((uint32_t)entryPtr[3] << 24);
        
        // Skip our own crypto_id
        if (entryCryptoId == cryptoId) {
            entryPtr += MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY;
            continue;
        }
        
        // Check if this crypto_id was already seen (duplicate)
        bool isDuplicate = false;
        for (uint8_t j = 0; j < seenCount; j++) {
            if (seenCryptoIds[j] == entryCryptoId) {
                isDuplicate = true;
                break;
            }
        }
        
        if (!isDuplicate && forwardCount < UPDATE_STATUS_MAX_ROBOTS) {
            seenCryptoIds[seenCount++] = entryCryptoId;
            entriesToForward[forwardCount++].data = entryPtr;
        }
        
        entryPtr += MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY;
    }
    
    // If no entries to forward, we're done
    if (forwardCount == 0) {
        Logger::debug(MODULE, "UPDATE_STATUS: nothing to forward after filtering");
        return;
    }
    
    // Second pass: split entries into packets of RF_MAX_ROBOTS_PER_PACKET each
    uint8_t entriesQueued = 0;
    while (entriesQueued < forwardCount) {
        // Check if queue is full (SPSC: one slot reserved)
        if (centralUpdateQueueFull()) {
            Logger::warning(MODULE, "Central update queue full, dropping remaining entries");
            break;
        }
        
        // Calculate how many entries for this packet
        uint8_t entriesThisPacket = forwardCount - entriesQueued;
        if (entriesThisPacket > RF_MAX_ROBOTS_PER_PACKET) {
            entriesThisPacket = RF_MAX_ROBOTS_PER_PACKET;
        }
        
        // Build packet
        CentralUpdateQueueItem& item = centralUpdateQueue[centralUpdateQueueHead];
        uint8_t* outPtr = item.data;
        
        // Copy data_type
        *outPtr++ = DATA_TYPE_UPDATE_STATUS;
        
        // Copy entries for this packet
        for (uint8_t i = 0; i < entriesThisPacket; i++) {
            memcpy(outPtr, entriesToForward[entriesQueued + i].data, MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY);
            outPtr += MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY;
        }
        
        // Calculate message length and checksum
        size_t msgLen = 1 + entriesThisPacket * MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY + 2;
        item.length = msgLen;
        
        uint16_t checksum = 0;
        for (size_t i = 0; i < msgLen - 2; i++) {
            checksum = (uint16_t)((checksum << 8) | (checksum >> 8));
            checksum ^= item.data[i];
        }
        item.data[msgLen - 2] = checksum & 0xFF;
        item.data[msgLen - 1] = (checksum >> 8) & 0xFF;
        
        // Memory barrier: ensure all queue item data is written before head becomes visible to consumer (Core 0)
        __sync_synchronize();
        // Commit to queue (advance head AFTER data is written and barrier)
        centralUpdateQueueHead = (centralUpdateQueueHead + 1) % CENTRAL_UPDATE_QUEUE_SIZE;
        entriesQueued += entriesThisPacket;
        
        Logger::infof(MODULE, "UPDATE_STATUS queued for RF: %d robot(s), %d bytes (packet %d)", 
                      entriesThisPacket, msgLen, (entriesQueued + RF_MAX_ROBOTS_PER_PACKET - 1) / RF_MAX_ROBOTS_PER_PACKET);
    }
    
    Logger::infof(MODULE, "Total: %d entries split into %d RF packets", 
                  forwardCount, (forwardCount + RF_MAX_ROBOTS_PER_PACKET - 1) / RF_MAX_ROBOTS_PER_PACKET);
}

void MyCC1101::sendPing() {
    static unsigned long lastPingTxMs = 0;
    static unsigned long constantOffset = 0;  // Per-device constant offset, set once at boot
    static unsigned long currentJitter = 0;
    static bool initialized = false;
    static unsigned long lastFallTimestamp = 0;  // millis() when robot last fell (0 = never)
    static bool wasFallen = false;               // Previous fall state for edge detection
    unsigned long now = millis();
    
    const unsigned long pingInterval = PING_INTERVAL;
    const unsigned long maxJitter = (unsigned long)(pingInterval * JITTER_FRACTION);
    
    // On first call, initialize with random offsets based on crypto_id
    // This prevents devices from synchronizing if they boot at similar times
    if (!initialized) {
        initialized = true;
        // Seed random with crypto_id for deterministic but unique per-device offset
        randomSeed(cryptoId ^ (cryptoId >> 16));
        // Generate constant offset that stays the same for this device's lifetime
        constantOffset = random(maxJitter);
        // Add random initial time offset (0 to PING_INTERVAL) to stagger startup
        lastPingTxMs = now - random(pingInterval);
        currentJitter = random(maxJitter);
        return;
    }
    
    // Track fall state transitions for last_fall_ms_ago and immediate ping trigger
    bool currentlyFallen = isFallDetected;
    if (currentlyFallen && !wasFallen) {
        // Rising edge: robot just fell — trigger immediate ping
        // Set triggeredPingTime in the past so the ping fires on the very next cycle (ASAP)
        triggeredPingTime = now - TRIGGERED_PING_DELAY_MS;
        Logger::info(MODULE, "Robot fell - triggering immediate ping");
    }
    if (currentlyFallen) {
        // Keep updating timestamp every cycle while fallen, so that after standing up
        // last_fall_ms_ago counts from the last moment we were down (not from when the fall started)
        lastFallTimestamp = now;
    }
    wasFallen = currentlyFallen;
    
    // Check if triggered ping is due (after UPDATE_STATUS applied or fall detected)
    bool isTriggeredPing = false;
    if (triggeredPingTime > 0 && now - triggeredPingTime >= TRIGGERED_PING_DELAY_MS) {
        isTriggeredPing = true;
        triggeredPingTime = 0;
    }
    
    // Effective interval = base + constant offset + per-cycle jitter
    // Add jitter to the interval to prevent devices from staying synchronized
    if (!isTriggeredPing && (now - lastPingTxMs < pingInterval + constantOffset + currentJitter)) {
        return;
    }

    lastPingTxMs = now;
    // Generate new random jitter for next ping cycle
    currentJitter = random(maxJitter);
    
    PingMessage msg;
    msg.data_type = DATA_TYPE_PING;
    msg.crypto_id = cryptoId;
    msg.robot_battery = batteryMvToEncoded(ADC::batteryMillivolts());
    msg.controller_type = static_cast<uint8_t>(controllerModel);
    msg.controller_battery = controllerBattery;
    
    // Copy robot name (null-padded)
    memset(msg.robot_name, 0, sizeof(msg.robot_name));
    const char* name = SerialProtocol::getRobotName();
    strncpy(msg.robot_name, name, sizeof(msg.robot_name));
    
    // Temperature sensor values (Celsius, INT8_MIN = unavailable)
    msg.temp_main = TemperatureSensor::getMainBoardTemp();
    msg.temp_right = TemperatureSensor::getRightBoardTemp();
    msg.temp_left = TemperatureSensor::getLeftBoardTemp();
    
    msg.firmware_version = FIRMWARE_VERSION;
    
    // is_relayed = 0 for own messages (not relayed from RF)
    msg.is_relayed = 0;
    
    // Encode current LED states (LEDs 2-11) using same encoding as UPDATE_STATUS
    // Format: bit 7 = 1 (LED on), lower 7 bits = R*25 + G*5 + B (R,G,B quantized to 0-4)
    // Quantization must match the protocol's threshold-based scheme exactly:
    //   0-31 -> 0, 32-95 -> 1, 96-159 -> 2, 160-223 -> 3, 224-255 -> 4
    auto quantize = [](uint8_t v) -> uint8_t {
        if (v < 32) return 0;
        if (v < 96) return 1;
        if (v < 160) return 2;
        if (v < 224) return 3;
        return 4;
    };
    for (int i = 0; i < UPDATE_STATUS_NUM_LEDS; i++) {
        CRGB color = LEDController::getLED(static_cast<LEDPositions>(i + 2));  // LEDs 2-11
        uint8_t r5 = quantize(color.r);
        uint8_t g5 = quantize(color.g);
        uint8_t b5 = quantize(color.b);
        msg.leds_encoded[i] = 0x80 | (r5 * 25 + g5 * 5 + b5);
    }
    
    // Encode current speed/torque coefficients (upper 4 = speed, lower 4 = torque)
    SharedData::SystemState sysState = SharedData::getSystemState();
    uint8_t speedNibble = (uint8_t)(constrain(sysState.speedCoefficient, 0.0f, 1.0f) * 15.0f + 0.5f);
    uint8_t torqueNibble = (uint8_t)(constrain(sysState.torqueCoefficient, 0.0f, 1.0f) * 15.0f + 0.5f);
    msg.speed_torque = (speedNibble << 4) | torqueNibble;
    
    // Milliseconds since last fall: 0 = never fallen, 1 = currently fallen, >1 = ms since last fall (standing now)
    if (currentlyFallen) {
        msg.last_fall_ms_ago = 1;  // Currently down — always report 1 while fallen
    } else if (lastFallTimestamp > 0) {
        msg.last_fall_ms_ago = max(2UL, now - lastFallTimestamp);  // Standing, min 2 to distinguish from "currently fallen"
    } else {
        msg.last_fall_ms_ago = 0;  // Never fallen
    }
    
    // Populate scan slots: collect active entries, sort by timestamp (earliest first), pack into ping
    // Also restore eye color if scan indication has been active long enough
    {
        // Collect active scan entries
        struct ActiveScan { uint32_t crypto_id; unsigned long timestamp; };
        ActiveScan active[SCAN_SLOTS_IN_PING];
        int activeCount = 0;
        
        for (int i = 0; i < SCAN_SLOTS_IN_PING; i++) {
            if (scanBuffer[i].crypto_id != 0 && (now - scanBuffer[i].timestamp < SCAN_REPORT_WINDOW_MS)) {
                active[activeCount++] = {scanBuffer[i].crypto_id, scanBuffer[i].timestamp};
            }
        }
        
        // Sort by timestamp ascending (earliest first) — simple insertion sort for 4 elements
        for (int i = 1; i < activeCount; i++) {
            ActiveScan key = active[i];
            int j = i - 1;
            while (j >= 0 && active[j].timestamp > key.timestamp) {
                active[j + 1] = active[j];
                j--;
            }
            active[j + 1] = key;
        }
        
        // Pack into ping message
        for (int i = 0; i < SCAN_SLOTS_IN_PING; i++) {
            msg.scan_crypto_ids[i] = (i < activeCount) ? active[i].crypto_id : 0;
        }
        
        // Restore eye color after scan indication expires (1.5s visual feedback, buffered for Core 1)
        if (scanIndicationActive && (now - scanIndicationStartTime >= 1500)) {
            pendingEyeR = savedEyeColorForScan.r;
            pendingEyeG = savedEyeColorForScan.g;
            pendingEyeB = savedEyeColorForScan.b;
            __sync_synchronize();
            pendingEyeColorSet = true;
            scanIndicationActive = false;
        }
    }
    
    msg.checksum = calculateChecksum(msg);
    
    if (isCentralConnected) {
        Logger::debugf(MODULE, "Sending PING to BLE, crypto_id=0x%08X, name=%s, battery_enc=%d, temps=[%d,%d,%d]C",
                  cryptoId, name, msg.robot_battery, msg.temp_main, msg.temp_right, msg.temp_left);
        BLEServer::sendPingToCentral(msg);
    }
    
    Logger::debugf(MODULE, "Sending PING to RF, crypto_id=0x%08X, name=%s, battery_enc=%d, temps=[%d,%d,%d]C",
                  cryptoId, name, msg.robot_battery, msg.temp_main, msg.temp_right, msg.temp_left);
    queueRfSend(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg), RF_POWER_LONG_RANGE);
    statsTxPing++;
}

void MyCC1101::sendScan() {
    if (!Scan::shouldSendScan()) {
        return;
    }
    Logger::debug(MODULE, "Sending SCAN at low power");
    
    ScanMessage msg;
    msg.data_type = DATA_TYPE_SCAN;
    msg.timestamp = millis();
    msg.crypto_id = cryptoId;
    msg.checksum = calculateChecksum(msg);
    
    queueRfSend(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg), RF_POWER_SHORT_RANGE);
    statsTxScan++;
}

// Record a scan event in the scan buffer (called when SCAN received with good RSSI)
// Dedup: if same scanner already recorded within SCAN_REPORT_WINDOW_MS, ignore.
// If buffer full with active entries, drop the new scan.
void MyCC1101::recordScanEvent(uint32_t scannerCryptoId) {
    unsigned long now = millis();
    
    // Check if this scanner is already in the buffer within the dedup window
    for (int i = 0; i < SCAN_SLOTS_IN_PING; i++) {
        if (scanBuffer[i].crypto_id == scannerCryptoId && scanBuffer[i].crypto_id != 0) {
            if (now - scanBuffer[i].timestamp < SCAN_REPORT_WINDOW_MS) {
                // Same scan session, ignore (dedup)
                return;
            } else {
                // Expired entry from same scanner — update as new session
                scanBuffer[i].timestamp = now;
                Logger::infof(MODULE, "Scan buffer: updated slot %d for 0x%08X (new session)", i, scannerCryptoId);
                return;
            }
        }
    }
    
    // Not in buffer — find an empty or expired slot
    for (int i = 0; i < SCAN_SLOTS_IN_PING; i++) {
        if (scanBuffer[i].crypto_id == 0 || (now - scanBuffer[i].timestamp >= SCAN_REPORT_WINDOW_MS)) {
            scanBuffer[i].crypto_id = scannerCryptoId;
            scanBuffer[i].timestamp = now;
            Logger::infof(MODULE, "Scan buffer: recorded 0x%08X in slot %d", scannerCryptoId, i);
            return;
        }
    }
    
    // All 3 slots are active — drop this scan
    Logger::warningf(MODULE, "Scan buffer full, dropping scan from 0x%08X", scannerCryptoId);
}

uint32_t MyCC1101::getCryptoId() {
    ATECCCrypto& crypto = ATECCCrypto::getInstance();
    
    if (!crypto.isInitialized()) {
        Logger::error(MODULE, "Crypto chip not initialized");
        return 0;
    }
    
    const uint8_t* publicKey = crypto.getPublicKey();
    if (publicKey == nullptr) {
        Logger::error(MODULE, "Failed to get crypto public key");
        return 0;
    }
    
    return (uint32_t)(publicKey[60] << 24) | (uint32_t)(publicKey[61] << 16) | 
           (uint32_t)(publicKey[62] << 8) | (uint32_t)(publicKey[63]);
}

void MyCC1101::processCentralUpdateQueue() {
    // Process one message per cycle to avoid blocking (SPSC consumer)
    if (centralUpdateQueueEmpty()) {
        return;
    }
    
    // Memory barrier: ensure we see the latest data written by producer (Core 1) before reading
    __sync_synchronize();
    CentralUpdateQueueItem& item = centralUpdateQueue[centralUpdateQueueTail];
    
    // Note: Our own data has already been applied in setUpdateStatusFromCentral()
    // The queued message has our entry stripped out, so just broadcast to RF
    
    Logger::debugf(MODULE, "Sending UPDATE_STATUS to RF: %d bytes", item.length);
    queueRfSend(item.data, item.length, RF_POWER_LONG_RANGE);
    statsTxUpdateStatus++;
    
    // Memory barrier before advancing tail, so producer sees we're done with this slot
    __sync_synchronize();
    // Advance tail (consumer only writes tail)
    centralUpdateQueueTail = (centralUpdateQueueTail + 1) % CENTRAL_UPDATE_QUEUE_SIZE;
}

void MyCC1101::queueRfSend(const uint8_t* payload, uint8_t len, int powerDbm) {
    // Check if queue is full
    if (rfTxQueueCount >= RF_TX_QUEUE_SIZE) {
        Logger::warning(MODULE, "RF TX queue full, dropping oldest message");
        // Drop oldest message by advancing tail
        rfTxQueueTail = (rfTxQueueTail + 1) % RF_TX_QUEUE_SIZE;
        rfTxQueueCount--;
    }
    
    // Add to queue at head position
    RfTxQueueItem& item = rfTxQueue[rfTxQueueHead];
    
    // Wrap payload with canaries
    item.data[0] = DATA_STRUCTURE_CANARY_START;
    memcpy(item.data + 1, payload, len);
    item.data[len + 1] = DATA_STRUCTURE_CANARY_END;
    item.length = len + 2;
    item.powerDbm = powerDbm;
    item.valid = true;
    
    rfTxQueueHead = (rfTxQueueHead + 1) % RF_TX_QUEUE_SIZE;
    rfTxQueueCount++;
    
    Logger::debugf(MODULE, "Queued RF TX: %d bytes", item.length);
}

void MyCC1101::processRfTxQueue() {
    // Process one message per cycle to avoid blocking
    if (rfTxQueueCount == 0) {
        return;
    }
    
    // Check for TX timeout - if isTransmitting has been true for too long,
    // the interrupt probably didn't fire. Reset and recover.
    if (isTransmitting && (millis() - transmitStartTime > TX_TIMEOUT_MS)) {
        Logger::warning(MODULE, "TX timeout - interrupt did not fire, recovering");
        statsErrTxTimeout++;
        isTransmitting = false;
        transmittedFlag = false;
        radio.standby();  // Force radio to known state
        radio.startReceive();
    }
    
    // Don't send if we're still transmitting
    if (isTransmitting) {
        return;
    }
    
    RfTxQueueItem& item = rfTxQueue[rfTxQueueTail];
    if (!item.valid) {
        // Skip invalid items
        rfTxQueueTail = (rfTxQueueTail + 1) % RF_TX_QUEUE_SIZE;
        rfTxQueueCount--;
        return;
    }
    
    // Send with the power level stored in the queue item
    int state = radio.finishReceive();
    if (state != RADIOLIB_ERR_NONE) {
        Logger::errorf(MODULE, "Failed to finish receive, code %d", state);
        return;
    }
    
    state = radio.setOutputPower(item.powerDbm);
    if (state != RADIOLIB_ERR_NONE) {
        Logger::errorf(MODULE, "Failed to set output power, code %d", state);
        return;
    }
    
    // Get message type for logging (data[0] is canary, data[1] is data_type)
    uint8_t msgType = item.data[1];
    const char* msgTypeName = "UNKNOWN";
    switch (msgType) {
        case DATA_TYPE_PING: msgTypeName = "PING"; break;
        case DATA_TYPE_UPDATE_STATUS: msgTypeName = "UPDATE_STATUS"; break;
        case DATA_TYPE_SCAN: msgTypeName = "SCAN"; break;
    }
    
    bool success = false;
    bool ccaFail = false;
    for (int i = 0; i < RF_TX_RETRIES; i++) {
        state = radio.startTransmit(item.data, item.length);
        if (state == RADIOLIB_ERR_NONE) {
            success = true;
            isTransmitting = true;  // Mark that we actually started a transmission
            transmitStartTime = millis();  // Record start time for timeout detection
            Logger::infof(MODULE, "TX started: %s (%d bytes, %ddBm)", msgTypeName, item.length, item.powerDbm);
            break;
        }
        
        if (radio.wasLastTxBlockedByCca()) {
            ccaFail = true;
        }
    }
    if (!success) {
        if (ccaFail) {
            Logger::warningf(MODULE,
                "TX dropped %s: CCA blocked (after %d tries). CCA-block count=%lu",
                msgTypeName, RF_TX_RETRIES, (unsigned long)radio.getCcaBlockedTxCount());
        } else {
            Logger::warningf(MODULE,
                "TX failed %s after %d retries, last error: %d",
                msgTypeName, RF_TX_RETRIES, state);
        }
        statsErrTxFailed++;
        isTransmitting = false;
        radio.startReceive();
    }
    
    // Mark as processed
    item.valid = false;
    rfTxQueueTail = (rfTxQueueTail + 1) % RF_TX_QUEUE_SIZE;
    rfTxQueueCount--;
}
