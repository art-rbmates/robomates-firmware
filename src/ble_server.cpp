#include "ble_server.h"
#include "logger.h"
#include "cc1101.h"
#include "adc.h"
#include "controller_handler.h"
#include "led_controller.h"
#include "serial_protocol.h"
#include "subprogram.h"
#include "balance_controller.h"
#include "calibration_manager.h"
#include "controller_mapping.h"
#include "central_controller.h"
#include "melody_player.h"
#include "atecc_crypto.h"
#include <Arduino.h>
#include <string.h>
#include "btstack.h"

static const char* MODULE = "BLE";

// Static guards for one-time initialization
static bool g_attDbBuilt = false;
static bool g_advStarted = false;

// RX message queue for offloading work from BTstack callbacks
// Buffer must be large enough for max UPDATE_STATUS packet (243 bytes for 16 robots)
struct RxMsg { uint8_t buf[MSG_SIZE_MAX]; uint16_t len; };
static QueueHandle_t g_rxQueue = nullptr;

// TX message queue for outgoing BLE notifications
// Prevents silent message drops when a BLE notification is already in flight
struct BleTxMsg { uint8_t buf[MSG_SIZE_MAX]; uint16_t len; };
static QueueHandle_t g_bleTxQueue = nullptr;
#define BLE_TX_QUEUE_SIZE 16
static volatile bool g_txDrainScheduled = false;  // Prevent flooding BT dispatch queue

static void flushBleTxQueue() {
    if (g_bleTxQueue) xQueueReset(g_bleTxQueue);
}

// --- BTstack main-thread dispatcher (no malloc, ordered, reentrant-safe) ---
typedef void (*bt_fn_t)();
static QueueHandle_t g_btFnQ = nullptr;
static btstack_context_callback_registration_t g_btRunnerReg;
static volatile bool g_btRunnerScheduled = false;

uint8_t adv_data[31] = {0};
uint8_t adv_data_length = 0;

static char     g_advName[MAX_BLE_NAME_LEN];
static uint8_t  g_advNameLen = 0;

static void _bt_runner(void* /*ctx*/) {
    // Run exactly one function per tick, then reschedule if more remain.
    bt_fn_t fn = nullptr;
    if (xQueueReceive(g_btFnQ, &fn, 0) == pdTRUE && fn) {
        fn();
    }
    // If there are more, schedule again
    if (uxQueueMessagesWaiting(g_btFnQ) > 0) {
        btstack_run_loop_execute_on_main_thread(&g_btRunnerReg);
    } else {
        g_btRunnerScheduled = false;
    }
}

static void on_bt_thread(bt_fn_t fn){
    if (!g_btFnQ) {
        // create once; capacity 8 is plenty for your usage
        g_btFnQ = xQueueCreate(8, sizeof(bt_fn_t));
        g_btRunnerReg.callback = &_bt_runner;
        g_btRunnerReg.context  = nullptr;
    }
    // enqueue (drop if full to avoid deadlocks)
    BaseType_t ok = xQueueSend(g_btFnQ, &fn, 0);
    if (ok != pdTRUE) {
        Logger::debug(MODULE, "BT dispatch queue full; dropping task");
    }

    // schedule runner if not already scheduled
    if (!g_btRunnerScheduled) {
        g_btRunnerScheduled = true;
        btstack_run_loop_execute_on_main_thread(&g_btRunnerReg);
    }
}

// Static member initialization
bool BLEServer::connected = false;
uint16_t BLEServer::conHandle = HCI_CON_HANDLE_INVALID;
uint16_t BLEServer::txValueHandle = 0;
uint16_t BLEServer::rxValueHandle = 0;
bool BLEServer::txNotifyEnabled = false;

uint8_t BLEServer::messageBytes[MSG_SIZE_MAX] = {0};
size_t BLEServer::messageLen = 0;
bool BLEServer::pendingMessageUpdate = false;

uint8_t BLEServer::lastRx[256] = {0};
size_t BLEServer::lastRxLen = 0;

// Controller input streaming state
uint32_t BLEServer::controllerStreamEnabledAt = 0;
uint32_t BLEServer::lastControllerStreamSent = 0;

// Last BLE activity timestamp (for connection timeout detection)
static uint32_t lastBleActivity = 0;
static const uint32_t BLE_CONNECTION_TIMEOUT_MS = 5000;  // 5 seconds

// UUIDs (NUS-style)
static const uint8_t UUID_SERVICE[] = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E };
static const uint8_t UUID_TX_CHAR[] = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E };
static const uint8_t UUID_RX_CHAR[] = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E };

// BTstack registrations
static btstack_packet_callback_registration_t hciEventCb;
static btstack_context_callback_registration_t notifyCbReg;

// C-style callback wrappers (required by BTstack)
static void hciEventCallback(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    BLEServer::onHciEvent(packetType, channel, packet, size);
}

static void attEventCallback(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    BLEServer::onAttEvent(packetType, channel, packet, size);
}

static uint16_t attReadCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, 
                                uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    return BLEServer::attReadCb(con_handle, attribute_handle, offset, buffer, buffer_size);
}

static int attWriteCallback(hci_con_handle_t con_handle, uint16_t attribute_handle, 
                           uint16_t transaction_mode, uint16_t offset, 
                           uint8_t* buffer, uint16_t buffer_size) {
    return BLEServer::attWriteCb(con_handle, attribute_handle, transaction_mode, 
                                 offset, buffer, buffer_size);
}

static void notifyCallback(void* context) {
    BLEServer::onNotifyPossible(context);
}

void BLEServer::buildAdvData() {
    // Get name from SerialProtocol and add RBM_ prefix for BLE advertising
    const char* baseName = SerialProtocol::getRobotName();
    // Ensure baseName length plus prefix does not exceed MAX_BLE_NAME_LEN - 1 for null terminator
    size_t baseLen = strlen(baseName);
    size_t prefixLen = strlen(BLE_NAME_PREFIX);
    size_t maxBaseLen = MAX_BLE_NAME_LEN - 1 - prefixLen;  // Max chars for base name (excluding prefix and null)
    if (baseLen > maxBaseLen) {
        Logger::warningf(MODULE, "Base name truncated: %s (max %d chars)", baseName, maxBaseLen);
        baseLen = maxBaseLen;
    }
    snprintf(g_advName, sizeof(g_advName), "%s%.*s", BLE_NAME_PREFIX, (int)baseLen, baseName);
    g_advNameLen = (uint8_t)strlen(g_advName);
    Logger::infof(MODULE, "Advertising as: %s", g_advName);

    adv_data[adv_data_length++] = 2;                          // length (type + value)
    adv_data[adv_data_length++] = BLUETOOTH_DATA_TYPE_FLAGS;  // 0x01
    adv_data[adv_data_length++] = 0x06;

    adv_data[adv_data_length++] = (uint8_t)(g_advNameLen + 1); // +1 for type
    adv_data[adv_data_length++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME; // 0x09
    memcpy(&adv_data[adv_data_length], g_advName, g_advNameLen);
    adv_data_length += g_advNameLen;
}

void BLEServer::init() {   
    buildAdvData();

    // Create RX and TX queues once
    if (!g_rxQueue) g_rxQueue = xQueueCreate(8, sizeof(RxMsg));
    if (!g_bleTxQueue) g_bleTxQueue = xQueueCreate(BLE_TX_QUEUE_SIZE, sizeof(BleTxMsg));
    
    // Register HCI state handler
    hciEventCb.callback = &hciEventCallback;
    hci_add_event_handler(&hciEventCb);
    
    // No pairing for PoC
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(0);
    
    // Init notification callback registration
    notifyCbReg.callback = &notifyCallback;
    notifyCbReg.context = nullptr;
    
    Logger::info(MODULE, "Initialized (waiting for BTstack WORKING state)");

    // If BTstack is already up, do the one-time setup now (on the BT thread)
    uint8_t st = hci_get_state();
    if (st == HCI_STATE_WORKING) {
        on_bt_thread(+[](){
            if (!g_attDbBuilt) {
                Logger::info(MODULE, "BT already WORKING -> building GATT DB");
                setupAttDb();            // already on BT thread here
                g_attDbBuilt = true;
            }
            if (!g_advStarted) {
                Logger::info(MODULE, "BT already WORKING -> starting advertising");
                startAdvertising();      // use your existing function
                g_advStarted = true;
            }
        });
    }


}

bool BLEServer::isConnected() {
    return connected && (conHandle != HCI_CON_HANDLE_INVALID);
}

void BLEServer::sendMessage(const uint8_t* data, size_t len) {
    if (!connected || !txNotifyEnabled) return;
    if (len > sizeof(messageBytes)) {
        Logger::errorf(MODULE, "Message too large: %d > %d", len, sizeof(messageBytes));
        return;
    }
    
    // Push to TX queue (non-blocking)
    BleTxMsg m{};
    memcpy(m.buf, data, len);
    m.len = (uint16_t)len;
    if (!g_bleTxQueue || xQueueSend(g_bleTxQueue, &m, 0) != pdTRUE) {
        Logger::warning(MODULE, "BLE TX queue full, dropping message");
        return;
    }
    
    // If no notification is in flight, schedule drain on BT thread
    if (!pendingMessageUpdate && !g_txDrainScheduled) {
        g_txDrainScheduled = true;
        on_bt_thread(+[](){
            g_txDrainScheduled = false;
            if (pendingMessageUpdate) return;
            if (!connected || !txNotifyEnabled || conHandle == HCI_CON_HANDLE_INVALID) return;
            
            BleTxMsg txm;
            if (!g_bleTxQueue || xQueueReceive(g_bleTxQueue, &txm, 0) != pdTRUE) return;
            
            memcpy(messageBytes, txm.buf, txm.len);
            messageLen = txm.len;
            pendingMessageUpdate = true;
            
            uint8_t st = att_server_request_to_send_notification(&notifyCbReg, conHandle);
            if (st != ERROR_CODE_SUCCESS) {
                pendingMessageUpdate = false;
                if (st == ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER) {
                    connected = false;
                    conHandle = HCI_CON_HANDLE_INVALID;
                    txNotifyEnabled = false;
                }
            }
        });
    }
}

void BLEServer::sendPingToCentral(const MyCC1101::PingMessage& msg) {
    // Send the raw message struct
    sendMessage(reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

const uint8_t* BLEServer::getLastRx(size_t& len) {
    len = lastRxLen;
    return lastRx;
}

// Handle to disconnect (used by timeout handler)
static uint16_t g_handleToDisconnect = HCI_CON_HANDLE_INVALID;

void BLEServer::update() {
    // Check for BLE connection timeout (no data received for 5 seconds)
    if (connected && conHandle != HCI_CON_HANDLE_INVALID) {
        uint32_t now = millis();
        uint32_t timeSinceActivity = now - lastBleActivity;
        
        // Warning at 80% of timeout
        static bool timeoutWarningLogged = false;
        if (timeSinceActivity >= (BLE_CONNECTION_TIMEOUT_MS * 4 / 5) && !timeoutWarningLogged) {
            Logger::warningf(MODULE, "BLE connection stale: %lu ms without data (timeout at %u ms)", 
                           timeSinceActivity, BLE_CONNECTION_TIMEOUT_MS);
            timeoutWarningLogged = true;
        }
        
        if (timeSinceActivity >= BLE_CONNECTION_TIMEOUT_MS) {
            Logger::errorf(MODULE, "BLE CONNECTION TIMEOUT: %lu ms without any RX data - forcing disconnect (handle=0x%04X)", 
                           timeSinceActivity, conHandle);
            Logger::warningf(MODULE, "Central should send BLE_CMD_KEEPALIVE (0x%02X) at least every %u ms", 
                           BLE_CMD_KEEPALIVE, BLE_CONNECTION_TIMEOUT_MS / 2);
            
            // Store handle for BTstack thread and request disconnection
            g_handleToDisconnect = conHandle;
            on_bt_thread(+[](){
                if (g_handleToDisconnect != HCI_CON_HANDLE_INVALID) {
                    gap_disconnect(g_handleToDisconnect);
                    g_handleToDisconnect = HCI_CON_HANDLE_INVALID;
                }
            });
            
            // Clean up local state immediately
            connected = false;
            txNotifyEnabled = false;
            pendingMessageUpdate = false;
            flushBleTxQueue();
            conHandle = HCI_CON_HANDLE_INVALID;
            MyCC1101::setIsCentralConnected(false);
            timeoutWarningLogged = false;  // Reset for next connection
            
            Logger::info(MODULE, "Resuming advertising after connection timeout");
            resumeAdvertising();
            return;  // Skip RX processing this cycle
        }
        
        // Reset warning flag when activity is received
        if (timeSinceActivity < (BLE_CONNECTION_TIMEOUT_MS * 4 / 5)) {
            timeoutWarningLogged = false;
        }
    }
    
    // Process RX queue (offloaded from BTstack callback)
    if (g_rxQueue) {
        RxMsg m;
        while (xQueueReceive(g_rxQueue, &m, 0) == pdTRUE) {
            // Parse received data based on message type
            if (m.len < 1) continue;
            
            uint8_t dataType = m.buf[0];
            Logger::debugf(MODULE, "RX message type: 0x%02X, len: %d", dataType, m.len);
            
            if (dataType == DATA_TYPE_UPDATE_STATUS && 
                m.len >= MSG_SIZE_UPDATE_STATUS_MIN && 
                m.len <= MSG_SIZE_UPDATE_STATUS_MAX &&
                ((m.len - 3) % MSG_SIZE_UPDATE_STATUS_ROBOT_ENTRY) == 0) {
                // Multi-robot UPDATE_STATUS format
                uint8_t robotCount = MyCC1101::getUpdateStatusRobotCount(m.len);
                Logger::infof(MODULE, "Received UPDATE_STATUS for %d robot(s), len=%d", robotCount, m.len);
                
                // Forward to CC1101 for processing (applies own, forwards others via RF)
                MyCC1101::setUpdateStatusFromCentral(m.buf, m.len);
            } else if (dataType == BLE_CMD_LIST_SUBPROGRAMS) {
                uint8_t startIndex = (m.len >= 2) ? m.buf[1] : 0;
                handleListSubprograms(startIndex);
            } else if (dataType == BLE_CMD_CREATE_SUBPROGRAM && m.len >= 2) {
                handleCreateSubprogram(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_RENAME_SUBPROGRAM && m.len >= 3) {
                handleRenameSubprogram(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_UPDATE_SUBPROGRAM && m.len >= 4) {
                handleUpdateSubprogram(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_SET_SUBPROGRAM_BTN && m.len >= 3) {
                handleSetSubprogramBtn(m.buf[1], m.buf[2]);
            } else if (dataType == BLE_CMD_DELETE_SUBPROGRAM && m.len >= 2) {
                handleDeleteSubprogram(m.buf[1]);
            } else if (dataType == BLE_CMD_GET_SUBPROGRAM && m.len >= 2) {
                handleGetSubprogram(m.buf[1]);
            } else if (dataType == BLE_CMD_GET_PITCH_CONSTANT) {
                handleGetPitchConstant();
            } else if (dataType == BLE_CMD_SET_PITCH_CONSTANT && m.len >= 5) {
                handleSetPitchConstant(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_RECALIBRATE) {
                handleRecalibrate();
            } else if (dataType == BLE_CMD_READ_SUBPROGRAM_DATA && m.len >= 4) {
                handleReadSubprogramData(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_START_SUBPROGRAM_WRITE && m.len >= 4) {
                handleStartSubprogramWrite(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_WRITE_SUBPROGRAM_DATA && m.len >= 5) {
                handleWriteSubprogramData(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_FINISH_SUBPROGRAM_WRITE && m.len >= 2) {
                handleFinishSubprogramWrite(m.buf[1]);
            } else if (dataType == BLE_CMD_GET_CONTROLLER_MAPPING && m.len >= 2) {
                handleGetControllerMapping(m.buf[1]);
            } else if (dataType == BLE_CMD_SET_CONTROLLER_MAPPING && m.len >= 2) {
                handleSetControllerMapping(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_RESET_CONTROLLER_MAPPING && m.len >= 2) {
                handleResetControllerMapping(m.buf[1]);
            } else if (dataType == BLE_CMD_GET_STEERING_SENSITIVITY) {
                handleGetSteeringSensitivity();
            } else if (dataType == BLE_CMD_SET_STEERING_SENSITIVITY && m.len >= 9) {
                handleSetSteeringSensitivity(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_GET_VELOCITY_LIMITS) {
                handleGetVelocityLimits();
            } else if (dataType == BLE_CMD_SET_VELOCITY_LIMITS && m.len >= 9) {
                handleSetVelocityLimits(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_KEEPALIVE) {
                // Keepalive message from central - silently ignore (activity timer already updated)
            } else if (dataType == BLE_CMD_CONTROLLER_INPUT && m.len >= 2) {
                handleControllerInput(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_READ_MELODY_DATA && m.len >= 3) {
                handleReadMelodyData(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_START_MELODY_WRITE && m.len >= 3) {
                handleStartMelodyWrite(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_WRITE_MELODY_DATA && m.len >= 4) {
                handleWriteMelodyData(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_FINISH_MELODY_WRITE) {
                handleFinishMelodyWrite();
            } else if (dataType == BLE_CMD_RESET_MELODY) {
                handleResetMelody();
            } else if (dataType == BLE_CMD_GET_MELODY_AMPLITUDE) {
                handleGetMelodyAmplitude();
            } else if (dataType == BLE_CMD_SET_MELODY_AMPLITUDE && m.len >= 5) {
                handleSetMelodyAmplitude(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_GET_PUBLIC_KEY) {
                handleGetPublicKey();
            } else if (dataType == BLE_CMD_SIGN_MESSAGE && m.len >= 3) {
                handleSignMessage(m.buf + 1, m.len - 1);
            } else if (dataType == BLE_CMD_ENABLE_CONTROLLER_STREAM) {
                handleEnableControllerStream();
            } else {
                Logger::errorf(MODULE, "Unknown or invalid message: type=0x%02X, len=%d", dataType, m.len);
            }
        }
    }
    
    // Controller input streaming - send data every CONTROLLER_STREAM_INTERVAL_MS while enabled
    if (connected && txNotifyEnabled && controllerStreamEnabledAt > 0) {
        uint32_t now = millis();
        
        // Check if streaming is still within the enabled window
        if (now - controllerStreamEnabledAt < CONTROLLER_STREAM_DURATION_MS) {
            // Check if it's time to send the next packet
            if (now - lastControllerStreamSent >= CONTROLLER_STREAM_INTERVAL_MS) {
                sendControllerStreamData();
                lastControllerStreamSent = now;
            }
        } else {
            // Streaming duration expired - disable streaming
            controllerStreamEnabledAt = 0;
        }
    }
}

void BLEServer::stopAdvertising() {
    on_bt_thread(+[](){ 
        gap_advertisements_enable(0); 
        Logger::debug(MODULE, "Advertising stopped"); 
    });
}

// Internal helper: actual advertising logic (BT thread only)
static void startAdvertising_impl() {
    // Connectable undirected advertising
    const uint8_t  own_addr_type = 0;   // PUBLIC
    const uint16_t adv_int       = 0x00A0; // ~100 ms
    const uint8_t  adv_type      = 0;   // ADV_IND
    uint8_t        direct_addr[6] = {0};
    const uint8_t  channel_map   = 0x07; // 37|38|39
    const uint8_t  filter_policy = 0;    // allow any

    gap_advertisements_set_params(adv_int, adv_int, adv_type, own_addr_type,
                                  direct_addr, channel_map, filter_policy);

    gap_advertisements_set_data(adv_data_length, adv_data);
    gap_scan_response_set_data(adv_data_length, adv_data);

    gap_advertisements_enable(1);
    Logger::info(MODULE, "Advertising started (name in ADV)");
}


void BLEServer::resumeAdvertising() {
    on_bt_thread(+[](){ 
        if (!connected && !ControllerHandler::isConnected()) {
            startAdvertising_impl();
        }
    });
}

void BLEServer::rebuildAdvertising() {
    // Rebuild advertising data with new name and restart advertising
    on_bt_thread(+[](){
        // Stop current advertising
        gap_advertisements_enable(0);
        
        // Reset and rebuild adv data
        adv_data_length = 0;
        buildAdvData();
        
        // Update GAP device name in GATT DB
        // Note: Full GATT rebuild would require disconnect, so just update adv data
        
        // Restart advertising if not connected
        if (!connected && !ControllerHandler::isConnected()) {
            startAdvertising_impl();
            Logger::info(MODULE, "Advertising restarted with new name");
        } else {
            Logger::info(MODULE, "Name updated - will advertise with new name after disconnect");
        }
    });
}

void BLEServer::startAdvertising() {
    // Ensure this always runs on BT thread
    on_bt_thread(+[](){ 
        startAdvertising_impl();
    });
}

void BLEServer::setupAttDb() {
    att_db_util_init();
    
    // GAP service + Device Name
    att_db_util_add_service_uuid16(GAP_SERVICE_UUID);
    att_db_util_add_characteristic_uuid16(
        GAP_DEVICE_NAME_UUID,
        ATT_PROPERTY_READ,
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        (uint8_t*)g_advName,
        g_advNameLen
    );
    
    // Custom UART-like service
    att_db_util_add_service_uuid128(UUID_SERVICE);
    
    // TX characteristic (Read + Notify)
    txValueHandle = att_db_util_add_characteristic_uuid128(
        UUID_TX_CHAR,
        (uint16_t)(ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY | ATT_PROPERTY_DYNAMIC),
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        nullptr,
        0
    );
    
    // RX characteristic (Write / Write Without Response)
    rxValueHandle = att_db_util_add_characteristic_uuid128(
        UUID_RX_CHAR,
        (uint16_t)(ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_DYNAMIC),
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        nullptr,
        0
    );
    
    att_server_init(att_db_util_get_address(), &attReadCallback, &attWriteCallback);
    att_server_register_packet_handler(&attEventCallback);
    
    Logger::infof(MODULE, "GATT DB setup complete (TX=0x%04X, RX=0x%04X)", txValueHandle, rxValueHandle);
}

void BLEServer::onHciEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packetType != HCI_EVENT_PACKET) return;
    
    const uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE: {
            const uint8_t state = btstack_event_state_get_state(packet);
            Logger::infof(MODULE, "HCI state: %d (working: %d)", state, HCI_STATE_WORKING);
            if (state == HCI_STATE_WORKING) {
                // One-time GATT build + advertising (no double init)
                if (!g_attDbBuilt) { 
                    Logger::info(MODULE, "HCI state WORKING -> building GATT DB");
                    setupAttDb(); 
                    g_attDbBuilt = true; 
                }
                if (!g_advStarted) { 
                    Logger::info(MODULE, "Starting advertising");
                    startAdvertising_impl();  // Direct call - already on BT thread
                    g_advStarted = true; 
                }
            }
            break;
        }
        
        case HCI_EVENT_LE_META: {
            const uint8_t subevent = hci_event_le_meta_get_subevent_code(packet);
            if (subevent == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                conHandle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                connected = true;
                txNotifyEnabled = false;
                pendingMessageUpdate = false;
                lastBleActivity = millis();  // Reset activity timer on connect
                
                // Log current connection parameters
                uint16_t connInterval = hci_subevent_le_connection_complete_get_conn_interval(packet);
                uint16_t connLatency = hci_subevent_le_connection_complete_get_conn_latency(packet);
                uint16_t supervisionTimeout = hci_subevent_le_connection_complete_get_supervision_timeout(packet);
                Logger::infof(MODULE, "Central connected (handle=0x%04X, interval=%u (%.1fms), latency=%u, timeout=%u (%ums))", 
                             conHandle, connInterval, connInterval * 1.25f, connLatency, 
                             supervisionTimeout, supervisionTimeout * 10);
                
                // Request better connection parameters for Linux/BlueZ compatibility
                // Parameters: conn_interval_min, conn_interval_max (units of 1.25ms), latency, timeout (units of 10ms)
                // Request: 15-30ms interval, 0 latency, 6 second timeout
                uint16_t reqIntervalMin = 12;   // 12 * 1.25ms = 15ms
                uint16_t reqIntervalMax = 24;   // 24 * 1.25ms = 30ms  
                uint16_t reqLatency = 0;        // Don't skip any connection events
                uint16_t reqTimeout = 600;      // 600 * 10ms = 6 seconds
                
                gap_request_connection_parameter_update(conHandle, reqIntervalMin, reqIntervalMax, reqLatency, reqTimeout);
                Logger::infof(MODULE, "Requested conn params: interval=%u-%u (%.1f-%.1fms), latency=%u, timeout=%u (%ums)",
                             reqIntervalMin, reqIntervalMax, reqIntervalMin * 1.25f, reqIntervalMax * 1.25f,
                             reqLatency, reqTimeout, reqTimeout * 10);
                
                MyCC1101::setIsCentralConnected(true);
            } else if (subevent == HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE) {
                // Log when connection parameters are actually updated
                uint8_t status = hci_subevent_le_connection_update_complete_get_status(packet);
                uint16_t connInterval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
                uint16_t connLatency = hci_subevent_le_connection_update_complete_get_conn_latency(packet);
                uint16_t supervisionTimeout = hci_subevent_le_connection_update_complete_get_supervision_timeout(packet);
                
                if (status == 0) {
                    Logger::infof(MODULE, "Conn params updated: interval=%u (%.1fms), latency=%u, timeout=%u (%ums)",
                                 connInterval, connInterval * 1.25f, connLatency, 
                                 supervisionTimeout, supervisionTimeout * 10);
                } else {
                    Logger::warningf(MODULE, "Conn params update failed: status=0x%02X", status);
                }
            }
            break;
        }
        
        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            uint8_t status = hci_event_disconnection_complete_get_status(packet);
            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            uint16_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            
            connected = false;
            txNotifyEnabled = false;
            pendingMessageUpdate = false;
            flushBleTxQueue();
            
            // Log disconnect reason with human-readable description
            const char* reasonStr = "Unknown";
            switch (reason) {
                case 0x08: reasonStr = "Connection timeout"; break;
                case 0x13: reasonStr = "Remote user terminated"; break;
                case 0x14: reasonStr = "Remote device terminated (low resources)"; break;
                case 0x15: reasonStr = "Remote device terminated (power off)"; break;
                case 0x16: reasonStr = "Local host terminated"; break;
                case 0x1A: reasonStr = "Unsupported remote feature"; break;
                case 0x22: reasonStr = "LMP response timeout"; break;
                case 0x28: reasonStr = "Instant passed"; break;
                case 0x29: reasonStr = "Pairing with unit key not supported"; break;
                case 0x2A: reasonStr = "Different transaction collision"; break;
                case 0x3B: reasonStr = "Unacceptable connection parameters"; break;
                case 0x3D: reasonStr = "Connection rejected (no suitable channel)"; break;
                case 0x3E: reasonStr = "Advertising timeout"; break;
                case 0x3F: reasonStr = "Connection terminated (MIC failure)"; break;
                case 0x40: reasonStr = "Connection failed to establish"; break;
            }
            
            Logger::warningf(MODULE, "Central disconnected: reason=0x%02X (%s), status=0x%02X, handle=0x%04X", 
                           reason, reasonStr, status, handle);
            
            MyCC1101::setIsCentralConnected(false);
            conHandle = HCI_CON_HANDLE_INVALID;
            
            Logger::info(MODULE, "Resuming advertising");
            resumeAdvertising();
            break;
        }
        
        default:
            break;
    }
}

void BLEServer::onAttEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packetType != HCI_EVENT_PACKET) return;
    
    const uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case ATT_EVENT_MTU_EXCHANGE_COMPLETE: {
            const uint16_t mtu = att_event_mtu_exchange_complete_get_MTU(packet);
            Logger::infof(MODULE, "MTU exchange complete: %u", mtu);
            break;
        }
        
        default:
            break;
    }
}

void BLEServer::onNotifyPossible(void* context) {
    if (!connected || !txNotifyEnabled || !pendingMessageUpdate) {
        pendingMessageUpdate = false;
        return;
    }
    
    att_server_notify(conHandle, txValueHandle, messageBytes, messageLen);
    Logger::debug(MODULE, "Notification sent");
    pendingMessageUpdate = false;
    
    // Immediately drain next message from TX queue (already on BT thread)
    BleTxMsg m;
    if (g_bleTxQueue && xQueueReceive(g_bleTxQueue, &m, 0) == pdTRUE) {
        memcpy(messageBytes, m.buf, m.len);
        messageLen = m.len;
        pendingMessageUpdate = true;
        
        uint8_t st = att_server_request_to_send_notification(&notifyCbReg, conHandle);
        if (st != ERROR_CODE_SUCCESS) {
            pendingMessageUpdate = false;
            if (st == ERROR_CODE_UNKNOWN_CONNECTION_IDENTIFIER) {
                connected = false;
                conHandle = HCI_CON_HANDLE_INVALID;
                txNotifyEnabled = false;
            }
        }
    }
}

uint16_t BLEServer::attReadCb(uint16_t conHandle, uint16_t attributeHandle, uint16_t offset,
                              uint8_t* buffer, uint16_t bufferSize) {
    if (attributeHandle == txValueHandle) {
        // Return battery voltage encoded (0=2500mV, 255=4200mV)
        uint8_t batteryEncoded = MyCC1101::batteryMvToEncoded(ADC::batteryMillivolts());
        if (offset == 0 && bufferSize >= 1) {
            buffer[0] = batteryEncoded;
            return 1;
        }
    }
    return 0;
}

int BLEServer::attWriteCb(uint16_t conHandle, uint16_t attributeHandle, uint16_t transactionMode,
                          uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
    if (attributeHandle == rxValueHandle) {
        if (offset + bufferSize <= sizeof(lastRx)) {
            memcpy(lastRx + offset, buffer, bufferSize);
            lastRxLen = offset + bufferSize;
            lastBleActivity = millis();  // Update activity timer on data received
            
            Logger::debugf(MODULE, "RX write: %u bytes", bufferSize);
            
            // Offload work to queue (do not block BTstack callback)
            RxMsg m{};
            m.len = (uint16_t)min<size_t>(lastRxLen, sizeof(m.buf));
            memcpy(m.buf, lastRx, m.len);
            if (g_rxQueue) xQueueSend(g_rxQueue, &m, 0);   // non-blocking
        }
        return 0;
    }
    
    // Handle CCCD writes for TX notifications
    if (attributeHandle == txValueHandle + 1) {  // CCCD is handle + 1
        if (bufferSize >= 2) {
            uint16_t cccdValue = little_endian_read_16(buffer, 0);
            txNotifyEnabled = (cccdValue & 0x0001) != 0;
            lastBleActivity = millis();  // Update activity timer - CCCD write is valid connection activity
            Logger::infof(MODULE, "TX notifications %s", txNotifyEnabled ? "enabled" : "disabled");
        }
        return 0;
    }
    
    return 0;
}

// ----- Subprogram BLE Handlers -----

void BLEServer::handleListSubprograms(uint8_t startIndex) {
    Logger::debugf(MODULE, "BLE: LIST_SUBPROGRAMS startIndex=%u", startIndex);
    
    uint8_t count = Subprogram::getSubprogramCount();
    uint8_t btnIds[5];
    Subprogram::getButtonSubprograms(btnIds);
    
    // Response header: CMD(1) + STATUS(1) + COUNT(1) + BTN1-5(5) + START_INDEX(1) + RETURNED_COUNT(1) = 10 bytes
    const size_t headerLen = 10;
    // Max response size based on BLE_CHUNK_SIZE + header
    const size_t maxResponseLen = BLE_CHUNK_SIZE + headerLen;
    
    uint8_t response[maxResponseLen];
    size_t offset = headerLen;  // Start after header, fill entries first
    uint8_t returnedCount = 0;
    
    // Fill entries starting from startIndex, as many as fit
    for (uint8_t i = startIndex; i < count && offset < maxResponseLen; i++) {
        uint8_t id;
        char name[SUBPROGRAM_NAME_MAX_LEN + 1];
        uint16_t dataLen;
        if (Subprogram::getSubprogramInfo(i, id, name, dataLen)) {
            uint8_t nameLen = strlen(name);
            size_t entryLen = 4 + nameLen;  // ID(1) + NAME_LEN(1) + DATA_LEN_H(1) + DATA_LEN_L(1) + NAME
            
            // Check if this entry fits
            if (offset + entryLen > maxResponseLen) {
                break;  // Entry doesn't fit, stop here
            }
            
            response[offset++] = id;
            response[offset++] = nameLen;
            response[offset++] = (dataLen >> 8) & 0xFF;
            response[offset++] = dataLen & 0xFF;
            memcpy(&response[offset], name, nameLen);
            offset += nameLen;
            returnedCount++;
        }
    }
    
    // Fill in header
    response[0] = BLE_CMD_LIST_SUBPROGRAMS;
    response[1] = 0x00;  // Success
    response[2] = count;
    response[3] = btnIds[0];  // Button 1 subprogram ID
    response[4] = btnIds[1];  // Button 2 subprogram ID
    response[5] = btnIds[2];  // Button 3 subprogram ID
    response[6] = btnIds[3];  // Button 4 subprogram ID
    response[7] = btnIds[4];  // Button 5 subprogram ID
    response[8] = startIndex;
    response[9] = returnedCount;
    
    sendMessage(response, offset);
    Logger::debugf(MODULE, "Sent %u of %u subprograms (starting at %u)", returnedCount, count, startIndex);
}

void BLEServer::handleCreateSubprogram(const uint8_t* data, size_t len) {
    if (len < 1) {
        uint8_t response[] = {BLE_CMD_CREATE_SUBPROGRAM, 0x00, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t nameLen = data[0];
    if (nameLen == 0 || nameLen > SUBPROGRAM_NAME_MAX_LEN || len < 1 + nameLen) {
        uint8_t response[] = {BLE_CMD_CREATE_SUBPROGRAM, 0x00, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    char name[SUBPROGRAM_NAME_MAX_LEN + 1];
    memcpy(name, data + 1, nameLen);
    name[nameLen] = '\0';
    
    Logger::debugf(MODULE, "BLE: CREATE_SUBPROGRAM name=%s", name);
    
    uint8_t newId = Subprogram::createSubprogram(name);
    uint8_t response[] = {BLE_CMD_CREATE_SUBPROGRAM, newId, (uint8_t)(newId > 0 ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleRenameSubprogram(const uint8_t* data, size_t len) {
    if (len < 2) {
        uint8_t response[] = {BLE_CMD_RENAME_SUBPROGRAM, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t id = data[0];
    uint8_t nameLen = data[1];
    
    if (nameLen == 0 || nameLen > SUBPROGRAM_NAME_MAX_LEN || len < 2 + nameLen) {
        uint8_t response[] = {BLE_CMD_RENAME_SUBPROGRAM, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    char name[SUBPROGRAM_NAME_MAX_LEN + 1];
    memcpy(name, data + 2, nameLen);
    name[nameLen] = '\0';
    
    Logger::debugf(MODULE, "BLE: RENAME_SUBPROGRAM ID=%u, name=%s", id, name);
    
    bool success = Subprogram::renameSubprogram(id, name);
    uint8_t response[] = {BLE_CMD_RENAME_SUBPROGRAM, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleUpdateSubprogram(const uint8_t* data, size_t len) {
    if (len < 3) {
        uint8_t response[] = {BLE_CMD_UPDATE_SUBPROGRAM, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t id = data[0];
    uint16_t dataLen = (data[1] << 8) | data[2];
    
    Logger::debugf(MODULE, "BLE: UPDATE_SUBPROGRAM ID=%u, dataLen=%u", id, dataLen);
    
    bool success = false;
    if (len >= 3 + dataLen) {
        success = Subprogram::updateSubprogramData(id, data + 3, dataLen);
    }
    
    uint8_t response[] = {BLE_CMD_UPDATE_SUBPROGRAM, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleSetSubprogramBtn(uint8_t buttonNum, uint8_t id) {
    Logger::debugf(MODULE, "BLE: SET_SUBPROGRAM_BTN btn=%u, ID=%u", buttonNum, id);
    
    if (buttonNum < 1 || buttonNum > 5) {
        uint8_t response[] = {BLE_CMD_SET_SUBPROGRAM_BTN, 0x01};  // Error - invalid button number
        sendMessage(response, sizeof(response));
        return;
    }
    
    Subprogram::setButtonSubprogram(buttonNum, id);
    uint8_t response[] = {BLE_CMD_SET_SUBPROGRAM_BTN, 0x00};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleDeleteSubprogram(uint8_t id) {
    Logger::debugf(MODULE, "BLE: DELETE_SUBPROGRAM ID=%u", id);
    bool success = Subprogram::deleteSubprogram(id);
    uint8_t response[] = {BLE_CMD_DELETE_SUBPROGRAM, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleGetSubprogram(uint8_t id) {
    Logger::debugf(MODULE, "BLE: GET_SUBPROGRAM ID=%u", id);
    
    SubprogramEntry entry;
    bool found = Subprogram::getSubprogramById(id, entry);
    
    if (!found) {
        uint8_t response[] = {BLE_CMD_GET_SUBPROGRAM, 0x01, 0x00, 0x00};
        sendMessage(response, sizeof(response));
        return;
    }
    
    // Response: CMD(1) + STATUS(1) + ID(1) + NAME_LEN(1) + NAME + DATA_LEN_H(1) + DATA_LEN_L(1)
    // Note: Bytecode data is NOT included - use READ_SUBPROGRAM_DATA for chunked transfer
    uint8_t nameLen = strlen(entry.name);
    uint8_t response[32];  // Max: 6 + 16 (max name) = 22 bytes
    
    uint16_t offset = 0;
    response[offset++] = BLE_CMD_GET_SUBPROGRAM;
    response[offset++] = 0x00;  // Success
    response[offset++] = entry.id;
    response[offset++] = nameLen;
    memcpy(&response[offset], entry.name, nameLen);
    offset += nameLen;
    response[offset++] = (entry.dataLength >> 8) & 0xFF;
    response[offset++] = entry.dataLength & 0xFF;
    
    sendMessage(response, offset);
    
    Logger::infof(MODULE, "Sent subprogram metadata ID=%u, dataLen=%u", id, entry.dataLength);
}

void BLEServer::handleReadSubprogramData(const uint8_t* data, size_t len) {
    if (len < 3) {
        uint8_t response[] = {BLE_CMD_READ_SUBPROGRAM_DATA, 0x01, 0x00};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t id = data[0];
    uint16_t offset = (data[1] << 8) | data[2];
    
    Logger::debugf(MODULE, "BLE: READ_SUBPROGRAM_DATA ID=%u, offset=%u", id, offset);
    
    uint8_t response[4 + BLE_CHUNK_SIZE];  // CMD + STATUS + OFFSET_H + OFFSET_L + DATA
    uint8_t chunkData[BLE_CHUNK_SIZE];
    
    uint8_t bytesRead = Subprogram::readSubprogramData(id, offset, chunkData, BLE_CHUNK_SIZE);
    
    response[0] = BLE_CMD_READ_SUBPROGRAM_DATA;
    response[1] = (bytesRead > 0) ? 0x00 : 0x01;
    response[2] = (offset >> 8) & 0xFF;
    response[3] = offset & 0xFF;
    
    if (bytesRead > 0) {
        memcpy(&response[4], chunkData, bytesRead);
        sendMessage(response, 4 + bytesRead);
        Logger::debugf(MODULE, "Sent %u bytes from offset %u", bytesRead, offset);
    } else {
        sendMessage(response, 4);
    }
}

void BLEServer::handleStartSubprogramWrite(const uint8_t* data, size_t len) {
    if (len < 3) {
        uint8_t response[] = {BLE_CMD_START_SUBPROGRAM_WRITE, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t id = data[0];
    uint16_t totalLen = (data[1] << 8) | data[2];
    
    Logger::debugf(MODULE, "BLE: START_SUBPROGRAM_WRITE ID=%u, len=%u", id, totalLen);
    
    bool success = Subprogram::startSubprogramWrite(id, totalLen);
    
    uint8_t response[] = {BLE_CMD_START_SUBPROGRAM_WRITE, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleWriteSubprogramData(const uint8_t* data, size_t len) {
    if (len < 4) {  // ID + OFFSET_H + OFFSET_L + at least 1 byte
        uint8_t response[] = {BLE_CMD_WRITE_SUBPROGRAM_DATA, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t id = data[0];
    uint16_t offset = (data[1] << 8) | data[2];
    uint8_t chunkLen = len - 3;
    
    Logger::debugf(MODULE, "BLE: WRITE_SUBPROGRAM_DATA ID=%u, offset=%u, len=%u", id, offset, chunkLen);
    
    bool success = Subprogram::writeSubprogramChunk(id, offset, data + 3, chunkLen);
    
    uint8_t response[] = {BLE_CMD_WRITE_SUBPROGRAM_DATA, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleFinishSubprogramWrite(uint8_t id) {
    Logger::debugf(MODULE, "BLE: FINISH_SUBPROGRAM_WRITE ID=%u", id);
    
    bool success = Subprogram::finishSubprogramWrite(id);
    
    uint8_t response[] = {BLE_CMD_FINISH_SUBPROGRAM_WRITE, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleGetPitchConstant() {
    Logger::debug(MODULE, "BLE: GET_PITCH_CONSTANT");
    
    float currentValue = BalanceController::getTargetPitchConstant();
    float defaultValue = BalanceController::getDefaultTargetPitchConstant();
    
    // Convert floats to bytes (little-endian)
    uint8_t response[10];
    response[0] = BLE_CMD_GET_PITCH_CONSTANT;
    response[1] = 0x00;  // Success
    
    // Current value (4 bytes, little-endian float)
    memcpy(&response[2], &currentValue, sizeof(float));
    
    // Default value (4 bytes, little-endian float)
    memcpy(&response[6], &defaultValue, sizeof(float));
    
    sendMessage(response, sizeof(response));
    Logger::infof(MODULE, "Sent pitch constant: current=%.2f, default=%.2f", currentValue, defaultValue);
}

void BLEServer::handleSetPitchConstant(const uint8_t* data, size_t len) {
    if (len < 4) {
        Logger::error(MODULE, "BLE: SET_PITCH_CONSTANT - invalid data length");
        uint8_t response[] = {BLE_CMD_SET_PITCH_CONSTANT, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    // Read float from bytes (little-endian)
    float newValue;
    memcpy(&newValue, data, sizeof(float));
    
    Logger::debugf(MODULE, "BLE: SET_PITCH_CONSTANT value=%.2f", newValue);
    
    BalanceController::setTargetPitchConstant(newValue);
    
    uint8_t response[] = {BLE_CMD_SET_PITCH_CONSTANT, 0x00};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleGetSteeringSensitivity() {
    Logger::debug(MODULE, "BLE: GET_STEERING_SENSITIVITY");
    
    float currentMin = BalanceController::getMinSteeringSensitivity();
    float currentMax = BalanceController::getMaxSteeringSensitivity();
    float defaultMin = BalanceController::getDefaultMinSteeringSensitivity();
    float defaultMax = BalanceController::getDefaultMaxSteeringSensitivity();
    
    // Response: CMD(1) + STATUS(1) + CURRENT_MIN(4) + CURRENT_MAX(4) + DEFAULT_MIN(4) + DEFAULT_MAX(4) = 18 bytes
    uint8_t response[18];
    response[0] = BLE_CMD_GET_STEERING_SENSITIVITY;
    response[1] = 0x00;  // Success
    
    // Current values (little-endian float)
    memcpy(&response[2], &currentMin, sizeof(float));
    memcpy(&response[6], &currentMax, sizeof(float));
    
    // Default values (little-endian float)
    memcpy(&response[10], &defaultMin, sizeof(float));
    memcpy(&response[14], &defaultMax, sizeof(float));
    
    sendMessage(response, sizeof(response));
    Logger::infof(MODULE, "Sent steering sensitivity: current min=%.2f max=%.2f, default min=%.2f max=%.2f", 
                  currentMin, currentMax, defaultMin, defaultMax);
}

void BLEServer::handleSetSteeringSensitivity(const uint8_t* data, size_t len) {
    if (len < 8) {
        Logger::error(MODULE, "BLE: SET_STEERING_SENSITIVITY - invalid data length");
        uint8_t response[] = {BLE_CMD_SET_STEERING_SENSITIVITY, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    // Read floats from bytes (little-endian)
    float minValue, maxValue;
    memcpy(&minValue, data, sizeof(float));
    memcpy(&maxValue, data + 4, sizeof(float));
    
    Logger::debugf(MODULE, "BLE: SET_STEERING_SENSITIVITY min=%.2f max=%.2f", minValue, maxValue);
    
    BalanceController::setSteeringSensitivity(minValue, maxValue);
    
    uint8_t response[] = {BLE_CMD_SET_STEERING_SENSITIVITY, 0x00};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleGetVelocityLimits() {
    Logger::debug(MODULE, "BLE: GET_VELOCITY_LIMITS");
    
    float currentBalance = BalanceController::getMaxBalanceVelocityForward();
    float currentBoost = BalanceController::getMaxBoostVelocityForward();
    float defaultBalance = BalanceController::getDefaultMaxBalanceVelocityForward();
    float defaultBoost = BalanceController::getDefaultMaxBoostVelocityForward();
    
    // Response: CMD(1) + STATUS(1) + CURRENT_BALANCE(4) + CURRENT_BOOST(4) + DEFAULT_BALANCE(4) + DEFAULT_BOOST(4) = 18 bytes
    uint8_t response[18];
    response[0] = BLE_CMD_GET_VELOCITY_LIMITS;
    response[1] = 0x00;  // Success
    
    // Current values (little-endian float)
    memcpy(&response[2], &currentBalance, sizeof(float));
    memcpy(&response[6], &currentBoost, sizeof(float));
    
    // Default values (little-endian float)
    memcpy(&response[10], &defaultBalance, sizeof(float));
    memcpy(&response[14], &defaultBoost, sizeof(float));
    
    sendMessage(response, sizeof(response));
    Logger::infof(MODULE, "Sent velocity limits: current balance=%.2f boost=%.2f, default balance=%.2f boost=%.2f", 
                  currentBalance, currentBoost, defaultBalance, defaultBoost);
}

void BLEServer::handleSetVelocityLimits(const uint8_t* data, size_t len) {
    if (len < 8) {
        Logger::error(MODULE, "BLE: SET_VELOCITY_LIMITS - invalid data length");
        uint8_t response[] = {BLE_CMD_SET_VELOCITY_LIMITS, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    // Read floats from bytes (little-endian)
    float balanceVelocity, boostVelocity;
    memcpy(&balanceVelocity, data, sizeof(float));
    memcpy(&boostVelocity, data + 4, sizeof(float));
    
    Logger::debugf(MODULE, "BLE: SET_VELOCITY_LIMITS balance=%.2f boost=%.2f", balanceVelocity, boostVelocity);
    
    BalanceController::setVelocityLimits(balanceVelocity, boostVelocity);
    
    uint8_t response[] = {BLE_CMD_SET_VELOCITY_LIMITS, 0x00};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleRecalibrate() {
    Logger::info(MODULE, "BLE: RECALIBRATE - clearing calibration and rebooting");
    
    // Send success response before rebooting
    uint8_t response[] = {BLE_CMD_RECALIBRATE, 0x00};
    sendMessage(response, sizeof(response));
    
    // Give time for the response to be sent
    delay(100);
    
    // Clear calibration data
    CalibrationManager::resetCalibration();
    
    // Reboot the device
    Logger::info(MODULE, "Rebooting for recalibration...");
    delay(100);
    ESP.restart();
}

void BLEServer::handleGetControllerMapping(uint8_t controllerType) {
    Logger::debugf(MODULE, "BLE: GET_CONTROLLER_MAPPING type=%u", controllerType);
    
    ControllerTypeMapping mapping;
    bool success = ControllerMapping::getMapping(controllerType, mapping);
    bool isCustom = ControllerMapping::hasCustomMapping(controllerType);
    
    if (!success) {
        uint8_t response[] = {BLE_CMD_GET_CONTROLLER_MAPPING, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    // Response: CMD(1) + STATUS(1) + TYPE(1) + IS_CUSTOM(1) + MAPPING_DATA(26)
    // Mapping data: buttonActions[17] + moveAxis(1) + moveInv(1) + turnAxis(1) + turnInv(1) + speedAxis(1) + speedInv(1)
    uint8_t response[30];
    uint8_t offset = 0;
    
    response[offset++] = BLE_CMD_GET_CONTROLLER_MAPPING;
    response[offset++] = 0x00;  // Success
    response[offset++] = controllerType;
    response[offset++] = isCustom ? 0x01 : 0x00;
    
    // Button mappings (17 bytes)
    memcpy(&response[offset], mapping.buttonActions, CTRL_INPUT_BTN_COUNT);
    offset += CTRL_INPUT_BTN_COUNT;
    
    // Axis mappings (6 bytes)
    response[offset++] = mapping.moveMapping.sourceAxis;
    response[offset++] = mapping.moveMapping.inverted ? 0x01 : 0x00;
    response[offset++] = mapping.turnMapping.sourceAxis;
    response[offset++] = mapping.turnMapping.inverted ? 0x01 : 0x00;
    response[offset++] = mapping.speedBoostMapping.sourceAxis;
    response[offset++] = mapping.speedBoostMapping.inverted ? 0x01 : 0x00;
    
    sendMessage(response, offset);
    Logger::infof(MODULE, "Sent mapping for type %u (custom=%d)", controllerType, isCustom);
}

void BLEServer::handleSetControllerMapping(const uint8_t* data, size_t len) {
    // Expected: TYPE(1) + buttonActions[17] + moveAxis(1) + moveInv(1) + turnAxis(1) + turnInv(1) + speedAxis(1) + speedInv(1)
    // Total: 1 + 17 + 6 = 24 bytes
    if (len < 24) {
        Logger::errorf(MODULE, "BLE: SET_CONTROLLER_MAPPING - invalid length %u", len);
        uint8_t response[] = {BLE_CMD_SET_CONTROLLER_MAPPING, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint8_t controllerType = data[0];
    Logger::debugf(MODULE, "BLE: SET_CONTROLLER_MAPPING type=%u", controllerType);
    
    ControllerTypeMapping mapping;
    memset(&mapping, 0, sizeof(mapping));
    
    // Parse button mappings
    memcpy(mapping.buttonActions, &data[1], CTRL_INPUT_BTN_COUNT);
    
    // Parse axis mappings
    uint8_t offset = 1 + CTRL_INPUT_BTN_COUNT;
    mapping.moveMapping.sourceAxis = data[offset++];
    mapping.moveMapping.inverted = data[offset++] != 0;
    mapping.turnMapping.sourceAxis = data[offset++];
    mapping.turnMapping.inverted = data[offset++] != 0;
    mapping.speedBoostMapping.sourceAxis = data[offset++];
    mapping.speedBoostMapping.inverted = data[offset++] != 0;
    
    bool success = ControllerMapping::setMapping(controllerType, mapping);
    
    uint8_t response[] = {BLE_CMD_SET_CONTROLLER_MAPPING, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleResetControllerMapping(uint8_t controllerType) {
    Logger::debugf(MODULE, "BLE: RESET_CONTROLLER_MAPPING type=%u", controllerType);
    
    bool success = ControllerMapping::resetToDefault(controllerType);
    
    uint8_t response[] = {BLE_CMD_RESET_CONTROLLER_MAPPING, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::logHex(const char* prefix, const uint8_t* data, size_t len) {
    Logger::debugf(MODULE, "%s [%u]: (hex data)", prefix, len);
}

void BLEServer::handleControllerInput(const uint8_t* data, size_t len) {
    // Expected packet structure (14 bytes):
    // Axes (12 bytes, little-endian int16_t):
    //   axisX(2) + axisY(2) + axisRX(2) + axisRY(2) + l2Axis(2) + r2Axis(2)
    // Buttons (2 bytes, bitmask):
    //   bit 0: a, bit 1: b, bit 2: x, bit 3: y
    //   bit 4: l1, bit 5: r1, bit 6: up, bit 7: down
    //   bit 8: left, bit 9: right
    
    if (len < 14) {
        Logger::errorf(MODULE, "BLE: CONTROLLER_INPUT - invalid length %u (expected 14)", len);
        return;
    }
    
    CentralController::BleControllerInput input = {};
    
    // Parse axes (little-endian int16_t)
    memcpy(&input.axisX, &data[0], sizeof(int16_t));
    memcpy(&input.axisY, &data[2], sizeof(int16_t));
    memcpy(&input.axisRX, &data[4], sizeof(int16_t));
    memcpy(&input.axisRY, &data[6], sizeof(int16_t));
    memcpy(&input.l2Axis, &data[8], sizeof(int16_t));
    memcpy(&input.r2Axis, &data[10], sizeof(int16_t));
    
    // Parse button bitmask
    uint16_t buttons = 0;
    memcpy(&buttons, &data[12], sizeof(uint16_t));
    
    input.a = (buttons & (1 << 0)) != 0;
    input.b = (buttons & (1 << 1)) != 0;
    input.x = (buttons & (1 << 2)) != 0;
    input.y = (buttons & (1 << 3)) != 0;
    input.l1 = (buttons & (1 << 4)) != 0;
    input.r1 = (buttons & (1 << 5)) != 0;
    input.up = (buttons & (1 << 6)) != 0;
    input.down = (buttons & (1 << 7)) != 0;
    input.left = (buttons & (1 << 8)) != 0;
    input.right = (buttons & (1 << 9)) != 0;
    
    // Forward to CentralController
    CentralController::setBleControllerInput(input);
    
    Logger::debugf(MODULE, "BLE controller input: Y=%d, RX=%d, btns=0x%04X",
        input.axisY, input.axisRX, buttons);
}

// ----- Melody BLE Handlers -----

void BLEServer::handleReadMelodyData(const uint8_t* data, size_t len) {
    if (len < 2) {
        uint8_t response[] = {BLE_CMD_READ_MELODY_DATA, 0x01, 0x00, 0x00};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint16_t offset = (data[0] << 8) | data[1];
    
    Logger::debugf(MODULE, "BLE: READ_MELODY_DATA offset=%u", offset);
    
    uint8_t response[4 + BLE_CHUNK_SIZE];  // CMD + STATUS + OFFSET_H + OFFSET_L + DATA
    uint8_t chunkData[BLE_CHUNK_SIZE];
    
    uint8_t bytesRead = MelodyPlayer::readMelodyData(offset, chunkData, BLE_CHUNK_SIZE);
    
    response[0] = BLE_CMD_READ_MELODY_DATA;
    response[1] = (bytesRead > 0) ? 0x00 : 0x01;
    response[2] = (offset >> 8) & 0xFF;
    response[3] = offset & 0xFF;
    
    if (bytesRead > 0) {
        memcpy(&response[4], chunkData, bytesRead);
        sendMessage(response, 4 + bytesRead);
        Logger::debugf(MODULE, "Sent %u bytes from offset %u", bytesRead, offset);
    } else {
        sendMessage(response, 4);
    }
}

void BLEServer::handleStartMelodyWrite(const uint8_t* data, size_t len) {
    if (len < 2) {
        uint8_t response[] = {BLE_CMD_START_MELODY_WRITE, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint16_t totalLen = (data[0] << 8) | data[1];
    
    Logger::debugf(MODULE, "BLE: START_MELODY_WRITE len=%u", totalLen);
    
    bool success = MelodyPlayer::startMelodyWrite(totalLen);
    
    uint8_t response[] = {BLE_CMD_START_MELODY_WRITE, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleWriteMelodyData(const uint8_t* data, size_t len) {
    if (len < 3) {  // OFFSET_H + OFFSET_L + at least 1 byte
        uint8_t response[] = {BLE_CMD_WRITE_MELODY_DATA, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint16_t offset = (data[0] << 8) | data[1];
    uint8_t chunkLen = len - 2;
    
    Logger::debugf(MODULE, "BLE: WRITE_MELODY_DATA offset=%u, len=%u", offset, chunkLen);
    
    bool success = MelodyPlayer::writeMelodyChunk(offset, data + 2, chunkLen);
    
    uint8_t response[] = {BLE_CMD_WRITE_MELODY_DATA, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleFinishMelodyWrite() {
    Logger::debug(MODULE, "BLE: FINISH_MELODY_WRITE");
    
    bool success = MelodyPlayer::finishMelodyWrite();
    
    uint8_t response[] = {BLE_CMD_FINISH_MELODY_WRITE, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleResetMelody() {
    Logger::info(MODULE, "BLE: RESET_MELODY");
    
    bool success = MelodyPlayer::resetToDefaultMelody();
    
    uint8_t response[] = {BLE_CMD_RESET_MELODY, (uint8_t)(success ? 0x00 : 0x01)};
    sendMessage(response, sizeof(response));
}

void BLEServer::handleGetMelodyAmplitude() {
    Logger::debug(MODULE, "BLE: GET_MELODY_AMPLITUDE");
    
    float currentValue = MelodyPlayer::getAmplitude();
    float defaultValue = MelodyPlayer::getDefaultAmplitude();
    
    // Response: CMD(1) + STATUS(1) + CURRENT(4) + DEFAULT(4) = 10 bytes
    uint8_t response[10];
    response[0] = BLE_CMD_GET_MELODY_AMPLITUDE;
    response[1] = 0x00;  // Success
    
    // Current value (little-endian float)
    memcpy(&response[2], &currentValue, sizeof(float));
    
    // Default value (little-endian float)
    memcpy(&response[6], &defaultValue, sizeof(float));
    
    sendMessage(response, sizeof(response));
    Logger::infof(MODULE, "Sent melody amplitude: current=%.2f, default=%.2f", currentValue, defaultValue);
}

void BLEServer::handleSetMelodyAmplitude(const uint8_t* data, size_t len) {
    if (len < 4) {
        Logger::error(MODULE, "BLE: SET_MELODY_AMPLITUDE - invalid data length");
        uint8_t response[] = {BLE_CMD_SET_MELODY_AMPLITUDE, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    // Read float from bytes (little-endian)
    float newValue;
    memcpy(&newValue, data, sizeof(float));
    
    Logger::debugf(MODULE, "BLE: SET_MELODY_AMPLITUDE value=%.2f", newValue);
    
    MelodyPlayer::setAmplitude(newValue);
    
    uint8_t response[] = {BLE_CMD_SET_MELODY_AMPLITUDE, 0x00};
    sendMessage(response, sizeof(response));
}

// ----- Crypto BLE Handlers -----

void BLEServer::handleGetPublicKey() {
    Logger::debug(MODULE, "BLE: GET_PUBLIC_KEY");
    
    ATECCCrypto& crypto = ATECCCrypto::getInstance();
    
    // Response: CMD(1) + STATUS(1) + PUBLIC_KEY(64) = 66 bytes
    uint8_t response[66];
    response[0] = BLE_CMD_GET_PUBLIC_KEY;
    
    if (!crypto.isInitialized()) {
        Logger::error(MODULE, "Crypto not initialized");
        response[1] = 0x01;  // Error
        sendMessage(response, 2);
        return;
    }
    
    if (!crypto.copyPublicKey(&response[2])) {
        Logger::error(MODULE, "Failed to copy public key");
        response[1] = 0x01;  // Error
        sendMessage(response, 2);
        return;
    }
    
    response[1] = 0x00;  // Success
    sendMessage(response, sizeof(response));
    Logger::info(MODULE, "Sent 64-byte public key");
}

void BLEServer::handleSignMessage(const uint8_t* data, size_t len) {
    // Data format: MSG_LEN_H(1) + MSG_LEN_L(1) + MESSAGE(up to BLE_SIGN_MSG_MAX_LEN bytes)
    if (len < 2) {
        Logger::error(MODULE, "BLE: SIGN_MESSAGE - missing length bytes");
        uint8_t response[] = {BLE_CMD_SIGN_MESSAGE, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    uint16_t msgLen = (data[0] << 8) | data[1];
    
    Logger::debugf(MODULE, "BLE: SIGN_MESSAGE msgLen=%u", msgLen);
    
    if (msgLen == 0 || msgLen > SIGN_MSG_MAX_LEN || len < 2 + msgLen) {
        Logger::errorf(MODULE, "Invalid message length: %u (max %u)", msgLen, SIGN_MSG_MAX_LEN);
        uint8_t response[] = {BLE_CMD_SIGN_MESSAGE, 0x01};
        sendMessage(response, sizeof(response));
        return;
    }
    
    ATECCCrypto& crypto = ATECCCrypto::getInstance();
    
    // Response: CMD(1) + STATUS(1) + SIGNATURE(64) = 66 bytes
    uint8_t response[66];
    response[0] = BLE_CMD_SIGN_MESSAGE;
    
    if (!crypto.isInitialized()) {
        Logger::error(MODULE, "Crypto not initialized");
        response[1] = 0x01;  // Error
        sendMessage(response, 2);
        return;
    }
    
    // Sign the message (crypto library will hash it with SHA-256)
    if (!crypto.signRFData(data + 2, msgLen, &response[2])) {
        Logger::error(MODULE, "Failed to sign message");
        response[1] = 0x01;  // Error
        sendMessage(response, 2);
        return;
    }
    
    response[1] = 0x00;  // Success
    sendMessage(response, sizeof(response));
    Logger::infof(MODULE, "Signed %u-byte message, sent 64-byte signature", msgLen);
}

// ----- Controller Input Streaming Handlers -----

void BLEServer::handleEnableControllerStream() {
    Logger::debug(MODULE, "BLE: ENABLE_CONTROLLER_STREAM");
    
    // Reset the streaming enable timestamp
    controllerStreamEnabledAt = millis();
    
    // Send first packet immediately
    lastControllerStreamSent = 0;  // Force immediate send on next update()
}

void BLEServer::sendControllerStreamData() {
    // Build controller state packet
    // Format: CMD(1) + timestamp(4) + axisX(2) + axisY(2) + axisRX(2) + axisRY(2) 
    //         + brake(2) + throttle(2) + buttons(4) + controllerType(1) + controllerBattery(1)
    // Total: 23 bytes
    
    uint8_t response[23];
    uint8_t offset = 0;
    
    response[offset++] = BLE_CMD_ENABLE_CONTROLLER_STREAM;  // CMD
    
    // Timestamp (4 bytes, little-endian)
    uint32_t timestamp = millis();
    memcpy(&response[offset], &timestamp, sizeof(uint32_t));
    offset += 4;
    
    // Axes from physical controller (little-endian int16_t)
    int16_t axisX = (int16_t)ControllerHandler::getAxisX();
    int16_t axisY = (int16_t)ControllerHandler::getAxisY();
    int16_t axisRX = (int16_t)ControllerHandler::getAxisRX();
    int16_t axisRY = (int16_t)ControllerHandler::getAxisRY();
    int16_t brake = (int16_t)ControllerHandler::getL2Axis();
    int16_t throttle = (int16_t)ControllerHandler::getR2Axis();
    
    memcpy(&response[offset], &axisX, sizeof(int16_t));
    offset += 2;
    memcpy(&response[offset], &axisY, sizeof(int16_t));
    offset += 2;
    memcpy(&response[offset], &axisRX, sizeof(int16_t));
    offset += 2;
    memcpy(&response[offset], &axisRY, sizeof(int16_t));
    offset += 2;
    memcpy(&response[offset], &brake, sizeof(int16_t));
    offset += 2;
    memcpy(&response[offset], &throttle, sizeof(int16_t));
    offset += 2;
    
    // Button bitmask (4 bytes / 32 bits, little-endian)
    // All physical controller buttons
    uint32_t buttons = 0;
    if (ControllerHandler::isAButtonPressed())     buttons |= (1 << 0);
    if (ControllerHandler::isBButtonPressed())     buttons |= (1 << 1);
    if (ControllerHandler::isXButtonPressed())     buttons |= (1 << 2);
    if (ControllerHandler::isYButtonPressed())     buttons |= (1 << 3);
    if (ControllerHandler::isL1ButtonPressed())    buttons |= (1 << 4);
    if (ControllerHandler::isR1ButtonPressed())    buttons |= (1 << 5);
    if (ControllerHandler::isUpButtonPressed())    buttons |= (1 << 6);
    if (ControllerHandler::isDownButtonPressed())  buttons |= (1 << 7);
    if (ControllerHandler::isLeftButtonPressed())  buttons |= (1 << 8);
    if (ControllerHandler::isRightButtonPressed()) buttons |= (1 << 9);
    if (ControllerHandler::isL2ButtonPressed())    buttons |= (1 << 10);
    if (ControllerHandler::isR2ButtonPressed())    buttons |= (1 << 11);
    if (ControllerHandler::isL3ButtonPressed())    buttons |= (1 << 12);
    if (ControllerHandler::isR3ButtonPressed())    buttons |= (1 << 13);
    if (ControllerHandler::isStartButtonPressed()) buttons |= (1 << 14);
    if (ControllerHandler::isSelectButtonPressed())buttons |= (1 << 15);
    if (ControllerHandler::isMiscButtonPressed())  buttons |= (1 << 16);
    if (ControllerHandler::isConnected())          buttons |= (1 << 31);  // Controller connected flag
    
    memcpy(&response[offset], &buttons, sizeof(uint32_t));
    offset += 4;
    
    // Controller type and battery
    response[offset++] = (uint8_t)ControllerHandler::getControllerModel();
    response[offset++] = ControllerHandler::getControllerBattery();
    
    sendMessage(response, offset);
    
    Logger::debugf(MODULE, "Sent controller stream: Y=%d, RX=%d, btns=0x%08X, connected=%d",
        axisY, axisRX, buttons, ControllerHandler::isConnected());
}

