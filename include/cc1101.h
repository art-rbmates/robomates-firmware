#ifndef CC1101_H
#define CC1101_H

#include <Arduino.h>
#include "config.h"

// Queue sizes
#define RF_TX_QUEUE_SIZE 8
#define CENTRAL_UPDATE_QUEUE_SIZE 8

class MyCC1101 {
public:
    // ===== Message structures for the new protocol =====
    
    // Common header for all messages (used for minimum length check)
    struct __attribute__((packed)) MessageHeader {
        uint8_t data_type;      // 1 byte
    };
    
    // PING message: robot -> broadcast (57 bytes, no timestamp)
    // Contains: crypto_id, robot_battery (encoded mV), controller_type, controller_battery%, robot_name, temperature sensors, firmware_version, is_relayed, leds, speed/torque, last_fall_ms_ago, scan_crypto_ids(4)
    struct __attribute__((packed)) PingMessage {
        uint8_t data_type;          // 1 byte = DATA_TYPE_PING
        uint32_t crypto_id;         // 4 bytes
        uint8_t robot_battery;      // 1 byte (encoded mV: 0=2500mV, 255=4200mV)
        uint8_t controller_type;    // 1 byte
        uint8_t controller_battery; // 1 byte (percentage 0-100)
        char robot_name[ROBOT_NAME_MAX_LEN]; // 10 bytes (null-padded)
        int8_t temp_main;           // 1 byte - Main board temperature (Celsius, INT8_MIN = unavailable)
        int8_t temp_right;          // 1 byte - Right board temperature (Celsius, INT8_MIN = unavailable)
        int8_t temp_left;           // 1 byte - Left board temperature (Celsius, INT8_MIN = unavailable)
        uint16_t firmware_version;  // 2 bytes - Firmware version
        uint8_t is_relayed;         // 1 byte - 0 = original message from this robot, 1 = relayed from RF
        uint8_t leds_encoded[UPDATE_STATUS_NUM_LEDS]; // 10 bytes - Current LED state (LEDs 2-11, same encoding as UPDATE_STATUS)
        uint8_t speed_torque;       // 1 byte - Current speed/torque (upper 4=speed, lower 4=torque, 0-15 -> 0.0-1.0)
        uint32_t last_fall_ms_ago;  // 4 bytes - Milliseconds since last fall (0 = never fallen)
        uint32_t scan_crypto_ids[SCAN_SLOTS_IN_PING]; // 16 bytes - Crypto IDs of robots that scanned us recently (0 = empty), sorted earliest first
        uint16_t checksum;          // 2 bytes
    };
    
    // UPDATE_STATUS message: central -> robot(s)
    // Multi-robot format: can contain data for 1-16 robots in a single packet
    // Per-robot entry: crypto_id(4) + leds(10) + speed_torque(1) = 15 bytes
    // LEDs are indices 2-11 (excluding eyes), each 1 byte:
    //   - First bit = 0: LED off (skip, don't change)
    //   - First bit = 1: LED on, lower 7 bits = R*25 + G*5 + B (0-124)
    //   - R,G,B are 0-4, mapping to: 0, 63, 127, 191, 255
    // Speed/Torque: upper 4 bits = speed (0-15 -> 0.0-1.0), lower 4 bits = torque (0-15 -> 0.0-1.0)
    struct __attribute__((packed)) UpdateStatusRobotEntry {
        uint32_t crypto_id;         // 4 bytes (target robot)
        uint8_t leds[UPDATE_STATUS_NUM_LEDS]; // 10 bytes (LEDs 2-11, excluding eyes)
        uint8_t speed_torque;       // 1 byte (upper 4 bits = speed, lower 4 bits = torque)
    };
    
    // Full UPDATE_STATUS message structure (variable length)
    // data_type(1) + N * RobotEntry(15) + checksum(2)
    struct __attribute__((packed)) UpdateStatusMessage {
        uint8_t data_type;          // 1 byte = DATA_TYPE_UPDATE_STATUS
        UpdateStatusRobotEntry entries[UPDATE_STATUS_MAX_ROBOTS]; // Up to 16 robots
        // Note: checksum is at the end, after the actual number of entries
        // Actual message size depends on number of robots: 1 + N*15 + 2
    };
    
    // Helper to get number of robots in an UPDATE_STATUS message from its total length
    static inline uint8_t getUpdateStatusRobotCount(size_t msgLen) {
        if (msgLen < MSG_SIZE_UPDATE_STATUS_MIN) return 0;
        return (msgLen - 3) / MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY; // (len - data_type - checksum) / entry_size
    }
    
    // Helper to get checksum position in UPDATE_STATUS message
    static inline size_t getUpdateStatusChecksumOffset(uint8_t robotCount) {
        return 1 + robotCount * MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY;
    }
    
    // SCAN message: robot -> broadcast (low power)
    // Contains: crypto_id
    struct __attribute__((packed)) ScanMessage {
        uint8_t data_type;          // 1 byte = DATA_TYPE_SCAN
        uint32_t timestamp;         // 4 bytes
        uint32_t crypto_id;         // 4 bytes
        uint16_t checksum;          // 2 bytes
    };
    
    // RF TX Queue item - stores pending RF transmissions
    struct RfTxQueueItem {
        uint8_t data[MSG_SIZE_MAX + 2];  // Max message size + 2 canaries
        uint8_t length;
        int8_t powerDbm;
        bool valid;
    };
    
    struct Stats {
        // RX counters by message type
        uint32_t rx_ping;
        uint32_t rx_update_status;
        uint32_t rx_scan;
        
        // TX counters by message type
        uint32_t tx_ping;
        uint32_t tx_update_status;
        uint32_t tx_scan;
        
        // Error counters
        uint32_t err_crc;           // CRC mismatch
        uint32_t err_validation;    // Data validation failed
        uint32_t err_zero_length;   // Zero-length packet
        uint32_t err_read_failed;   // Failed to read data
        uint32_t err_tx_failed;     // TX failed after retries
        uint32_t err_tx_timeout;    // TX timeout (interrupt didn't fire)
        
        // Total helpers
        uint32_t totalRx() const { return rx_ping + rx_update_status + rx_scan; }
        uint32_t totalTx() const { return tx_ping + tx_update_status + tx_scan; }
        uint32_t totalErrors() const { return err_crc + err_validation + err_zero_length + err_read_failed + err_tx_failed + err_tx_timeout; }
    };

    static void init();
    static void update();
    static void setIsCentralConnected(bool isCentralConnected);
    static void setIsFallDetected(bool isFallDetected);
    static void setControllerBattery(uint8_t battery);
    static void setControllerModel(int model);
    
    // Statistics: get current stats and reset counters
    static Stats getAndResetStats();
    
    // Handle UPDATE_STATUS from central (via BLE) - multi-robot format
    // data points to the full message starting with data_type byte
    // len is the total message length (18-243 bytes for 1-16 robots)
    static void setUpdateStatusFromCentral(const uint8_t* data, size_t len);

    static uint32_t getCryptoId();
    
    // Battery conversion: millivolts to encoded byte (0=2500mV, 255=4200mV)
    static uint8_t batteryMvToEncoded(uint16_t millivolts);
    
    // Pending LED update from RF path (Core 0 writes, Core 1 applies)
    // Call applyPendingLedUpdate() from the main loop (Core 1) to apply buffered LED changes
    static void applyPendingLedUpdate();
    
private:
    // Pending LED buffer: stores decoded LED colors from RF-received UPDATE_STATUS
    // Written by Core 0, read and cleared by Core 1 via applyPendingLedUpdate()
    struct PendingLedEntry {
        uint8_t ledIndex;  // Actual LED index (2-11)
        uint8_t r, g, b;
    };
    static PendingLedEntry pendingLeds[UPDATE_STATUS_NUM_LEDS];
    static volatile uint8_t pendingLedCount;  // 0 = no pending update
    
    // Pending eye color from scan indication on Core 0
    static volatile bool pendingEyeColorSet;
    static uint8_t pendingEyeR, pendingEyeG, pendingEyeB;
    // Connection state
    static volatile bool isCentralConnected;
    static volatile bool isFallDetected;
    static volatile uint8_t controllerBattery;
    static volatile int controllerModel;
    
    // Triggered ping: timestamp when UPDATE_STATUS was applied (0 = not triggered)
    static unsigned long triggeredPingTime;
    
    // Statistics - RX by message type
    static volatile uint32_t statsRxPing;
    static volatile uint32_t statsRxUpdateStatus;
    static volatile uint32_t statsRxScan;
    
    // Statistics - TX by message type
    static volatile uint32_t statsTxPing;
    static volatile uint32_t statsTxUpdateStatus;
    static volatile uint32_t statsTxScan;
    
    // Statistics - Errors
    static volatile uint32_t statsErrCrc;
    static volatile uint32_t statsErrValidation;
    static volatile uint32_t statsErrZeroLength;
    static volatile uint32_t statsErrReadFailed;
    static volatile uint32_t statsErrTxFailed;
    static volatile uint32_t statsErrTxTimeout;
    
    // Queue for UPDATE_STATUS messages from central (variable length)
    // SPSC ring buffer: producer (BLE/Core 1) writes head, consumer (CC1101/Core 0) writes tail.
    // Count is derived from head/tail to avoid shared counter race condition.
    // Max usable capacity = CENTRAL_UPDATE_QUEUE_SIZE - 1 (one slot reserved to distinguish full from empty).
    struct CentralUpdateQueueItem {
        uint8_t data[MSG_SIZE_UPDATE_STATUS_MAX];  // Raw message bytes
        uint8_t length;                             // Actual message length
    };
    static CentralUpdateQueueItem centralUpdateQueue[CENTRAL_UPDATE_QUEUE_SIZE];
    static volatile uint8_t centralUpdateQueueHead;  // Written only by producer (Core 1)
    static volatile uint8_t centralUpdateQueueTail;  // Written only by consumer (Core 0)
    
    // Thread-safe helpers for SPSC ring buffer
    static inline bool centralUpdateQueueFull() {
        return ((centralUpdateQueueHead + 1) % CENTRAL_UPDATE_QUEUE_SIZE) == centralUpdateQueueTail;
    }
    static inline bool centralUpdateQueueEmpty() {
        return centralUpdateQueueHead == centralUpdateQueueTail;
    }
    
    // Queue for RF transmissions
    static RfTxQueueItem rfTxQueue[RF_TX_QUEUE_SIZE];
    static volatile uint8_t rfTxQueueHead;
    static volatile uint8_t rfTxQueueTail;
    static volatile uint8_t rfTxQueueCount;
    
    // Crypto ID (hardware ID)
    static uint32_t cryptoId;

    // Validation
    static uint8_t isValidData(const uint8_t* data, size_t length);
    static size_t getExpectedMessageSize(uint8_t dataType);

    // Checksum calculation (template for any message type)
    template<typename T>
    static uint16_t calculateChecksum(const T& msg) {
        uint16_t c = 0;
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&msg);
        // Calculate checksum over all bytes except the last 2 (checksum field)
        for (size_t i = 0; i < sizeof(T) - sizeof(uint16_t); ++i) {
            c = (uint16_t)((c << 8) | (c >> 8));
            c ^= p[i];
        }
        return c;
    }

    // Send specific message types
    static void sendPing();
    static void sendScan();
    
    // Scan result buffer: stores recent scan events to embed in pings
    struct ScanBufferEntry {
        uint32_t crypto_id;     // Scanner's crypto ID (0 = empty)
        unsigned long timestamp; // When first detected in this session
    };
    static ScanBufferEntry scanBuffer[SCAN_SLOTS_IN_PING];
    static void recordScanEvent(uint32_t scannerCryptoId);

    // Access to connection state
    static bool getIsCentralConnected() { return isCentralConnected; }
    
    // Message handling
    static void handleRxPayload(const uint8_t* data, size_t length, int8_t rssi);
    static void handlePing(const PingMessage& msg, int8_t rssi);
    // Handle UPDATE_STATUS - applies to this robot if crypto_id matches, returns true if applied
    static bool handleUpdateStatus(const uint8_t* data, size_t len, int8_t rssi);
    static void handleScan(const ScanMessage& msg, int8_t rssi);
    
    // RF transmission queue
    static void queueRfSend(const uint8_t* payload, uint8_t len, int powerDbm);
    static void processRfTxQueue();
    
    // Central update queue
    static void processCentralUpdateQueue();
};

#endif // CC1101_H

