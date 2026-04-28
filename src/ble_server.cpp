#include "ble_server.h"
#include "logger.h"
#include "serial_protocol.h"
#include "base_logic.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>
#include "btstack.h"

static const char* MODULE = "BLE";

static bool g_attDbBuilt = false;
static bool g_advStarted = false;

typedef void (*bt_fn_t)();
static QueueHandle_t g_btFnQ = nullptr;
static btstack_context_callback_registration_t g_btRunnerReg;
static volatile bool g_btRunnerScheduled = false;

uint8_t adv_data[31] = {0};
uint8_t adv_data_length = 0;
uint8_t scan_resp_data[31] = {0};
uint8_t scan_resp_data_length = 0;

static char     g_advName[MAX_BLE_NAME_LEN];
static uint8_t  g_advNameLen = 0;

static uint32_t lastGattStatusMs = 0;

// Base service UUIDs (last byte 0x6F to distinguish from robot's 0x6E)
static const uint8_t UUID_BASE_SERVICE[]    = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6F };
static const uint8_t UUID_BASE_COLOR_CHAR[] = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6F };
static const uint8_t UUID_BASE_RX_CHAR[]    = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6F };
static const uint8_t UUID_BASE_TX_CHAR[]    = { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5,0x04,0x00,0x40,0x6F };

bool BLEServer::connected = false;
uint16_t BLEServer::conHandle = HCI_CON_HANDLE_INVALID;
uint16_t BLEServer::colorCharHandle = 0;
uint16_t BLEServer::rxCharHandle = 0;
uint16_t BLEServer::txCharHandle = 0;
bool BLEServer::txNotifyEnabled = false;
uint32_t BLEServer::lastActivity = 0;

uint8_t BLEServer::messageBytes[32] = {0};
size_t BLEServer::messageLen = 0;
bool BLEServer::pendingMessageUpdate = false;

static btstack_packet_callback_registration_t hciEventCb;
static btstack_context_callback_registration_t notifyCbReg;

static void _bt_runner(void* /*ctx*/) {
    bt_fn_t fn = nullptr;
    if (xQueueReceive(g_btFnQ, &fn, 0) == pdTRUE && fn) {
        fn();
    }
    if (uxQueueMessagesWaiting(g_btFnQ) > 0) {
        btstack_run_loop_execute_on_main_thread(&g_btRunnerReg);
    } else {
        g_btRunnerScheduled = false;
    }
}

static void on_bt_thread(bt_fn_t fn){
    if (!g_btFnQ) {
        g_btFnQ = xQueueCreate(8, sizeof(bt_fn_t));
        g_btRunnerReg.callback = &_bt_runner;
        g_btRunnerReg.context  = nullptr;
    }
    BaseType_t ok = xQueueSend(g_btFnQ, &fn, 0);
    if (ok != pdTRUE) {
        Logger::debug(MODULE, "BT dispatch queue full; dropping task");
    }
    if (!g_btRunnerScheduled) {
        g_btRunnerScheduled = true;
        btstack_run_loop_execute_on_main_thread(&g_btRunnerReg);
    }
}

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

// ---- Advertising data ----

void BLEServer::buildAdvData() {
    const char* baseName = SerialProtocol::getRobotName();
    size_t baseLen = strlen(baseName);
    size_t prefixLen = strlen(BLE_NAME_PREFIX);
    size_t maxBaseLen = MAX_BLE_NAME_LEN - 1 - prefixLen;
    if (baseLen > maxBaseLen) baseLen = maxBaseLen;

    snprintf(g_advName, sizeof(g_advName), "%s%.*s", BLE_NAME_PREFIX, (int)baseLen, baseName);
    g_advNameLen = (uint8_t)strlen(g_advName);
    Logger::infof(MODULE, "Advertising as: %s", g_advName);

    adv_data_length = 0;

    // Flags AD (3 bytes)
    adv_data[adv_data_length++] = 2;
    adv_data[adv_data_length++] = BLUETOOTH_DATA_TYPE_FLAGS;
    adv_data[adv_data_length++] = 0x06;

    // Manufacturer Specific Data AD
    adv_data[adv_data_length++] = 1 + 2 + ADV_BASE_STATUS_SIZE;
    adv_data[adv_data_length++] = 0xFF;
    adv_data[adv_data_length++] = (uint8_t)(ADV_MANUFACTURER_ID & 0xFF);
    adv_data[adv_data_length++] = (uint8_t)((ADV_MANUFACTURER_ID >> 8) & 0xFF);

    // Placeholder values; updateAdvStatusData() rewrites these every ADV_UPDATE_INTERVAL_MS.
    uint8_t statusOffset = adv_data_length;
    adv_data[statusOffset + 0] = DATA_TYPE_BASE_ADV;
    adv_data[statusOffset + 1] = 0;                 // seq low
    adv_data[statusOffset + 2] = 0;                 // seq high
    adv_data[statusOffset + 3] = BASE_COLOR_NEUTRAL;
    adv_data[statusOffset + 4] = 0;                 // changeCount
    adv_data[statusOffset + 5] = 0;                 // scoreSeconds low
    adv_data[statusOffset + 6] = 0;                 // scoreSeconds high
    adv_data[statusOffset + 7] = (uint8_t)(FIRMWARE_VERSION & 0xFF);
    adv_data[statusOffset + 8] = (uint8_t)((FIRMWARE_VERSION >> 8) & 0xFF);
    adv_data_length += ADV_BASE_STATUS_SIZE;

    Logger::infof(MODULE, "ADV data built: %d bytes", adv_data_length);
}

void BLEServer::buildScanResponseData() {
    scan_resp_data_length = 0;

    scan_resp_data[scan_resp_data_length++] = (uint8_t)(g_advNameLen + 1);
    scan_resp_data[scan_resp_data_length++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(&scan_resp_data[scan_resp_data_length], g_advName, g_advNameLen);
    scan_resp_data_length += g_advNameLen;

    Logger::infof(MODULE, "Scan response built: %d bytes (name: %s)", scan_resp_data_length, g_advName);
}

void BLEServer::updateAdvStatusData(uint8_t baseColor, uint8_t changeCount, uint16_t scoreSeconds) {
    const uint8_t statusOffset = 7;

    // 16-bit seq counter sits right after the type byte so it lands inside the
    // first ~8 bytes Android BLE controllers tend to hash for ADV de-duplication.
    // 16-bit width also avoids 256-tick aliasing at 200 ms cadence (51 s wrap on
    // u8 was short enough to coincide with Android's controller cache TTL).
    static uint16_t seqCounter = 0;
    seqCounter++;

    adv_data[statusOffset + 0] = DATA_TYPE_BASE_ADV;
    adv_data[statusOffset + 1] = (uint8_t)(seqCounter & 0xFF);
    adv_data[statusOffset + 2] = (uint8_t)((seqCounter >> 8) & 0xFF);
    adv_data[statusOffset + 3] = baseColor;
    adv_data[statusOffset + 4] = changeCount;
    adv_data[statusOffset + 5] = (uint8_t)(scoreSeconds & 0xFF);
    adv_data[statusOffset + 6] = (uint8_t)((scoreSeconds >> 8) & 0xFF);
    adv_data[statusOffset + 7] = (uint8_t)(FIRMWARE_VERSION & 0xFF);
    adv_data[statusOffset + 8] = (uint8_t)((FIRMWARE_VERSION >> 8) & 0xFF);

    on_bt_thread(+[](){
        gap_advertisements_set_data(adv_data_length, adv_data);
    });
}

// ---- Advertising control ----

void BLEServer::setAdvertisingConnectable(bool connectable) {
    static volatile bool s_connectable = true;
    s_connectable = connectable;

    on_bt_thread(+[](){
        gap_advertisements_enable(0);

        const uint8_t own_addr_type = 0;
        const uint16_t adv_int = ADV_INTERVAL;
        const uint8_t adv_type = s_connectable ? 0 : 2;  // ADV_IND vs ADV_SCAN_IND
        uint8_t direct_addr[6] = {0};
        const uint8_t channel_map = 0x07;
        const uint8_t filter_policy = 0;

        gap_advertisements_set_params(adv_int, adv_int, adv_type, own_addr_type,
                                      direct_addr, channel_map, filter_policy);
        gap_advertisements_set_data(adv_data_length, adv_data);
        gap_scan_response_set_data(scan_resp_data_length, scan_resp_data);
        gap_advertisements_enable(1);

        Logger::infof(MODULE, "Advertising mode: %s", s_connectable ? "connectable" : "non-connectable");
    });
}

static void startAdvertising_impl() {
    const uint8_t  own_addr_type = 0;
    const uint16_t adv_int       = ADV_INTERVAL;
    const uint8_t  adv_type      = 0;  // ADV_IND (connectable undirected)
    uint8_t        direct_addr[6] = {0};
    const uint8_t  channel_map   = 0x07;
    const uint8_t  filter_policy = 0;

    gap_advertisements_set_params(adv_int, adv_int, adv_type, own_addr_type,
                                  direct_addr, channel_map, filter_policy);
    gap_advertisements_set_data(adv_data_length, adv_data);
    gap_scan_response_set_data(scan_resp_data_length, scan_resp_data);
    gap_advertisements_enable(1);
    Logger::info(MODULE, "Advertising started (connectable, status in ADV, name in scan response)");
}

void BLEServer::startAdvertising() {
    on_bt_thread(+[](){ startAdvertising_impl(); });
}

void BLEServer::rebuildAdvertising() {
    on_bt_thread(+[](){
        gap_advertisements_enable(0);
        adv_data_length = 0;
        scan_resp_data_length = 0;
        buildAdvData();
        buildScanResponseData();
        startAdvertising_impl();
        Logger::info(MODULE, "Advertising restarted with new name");
    });
}

// ---- GATT notifications ----

void BLEServer::sendMessage(const uint8_t* data, size_t len) {
    if (!connected || !txNotifyEnabled || len > sizeof(messageBytes)) return;

    memcpy(messageBytes, data, len);
    messageLen = len;
    pendingMessageUpdate = true;

    on_bt_thread(+[](){
        if (!pendingMessageUpdate || !connected || !txNotifyEnabled ||
            conHandle == HCI_CON_HANDLE_INVALID) {
            pendingMessageUpdate = false;
            return;
        }
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

void BLEServer::onNotifyPossible(void* context) {
    if (!connected || !txNotifyEnabled || !pendingMessageUpdate) {
        pendingMessageUpdate = false;
        return;
    }

    att_server_notify(conHandle, txCharHandle, messageBytes, messageLen);
    pendingMessageUpdate = false;
}

void BLEServer::sendGattStatus() {
    if (!connected || !txNotifyEnabled) return;

    uint8_t msg[GATT_BASE_STATUS_MSG_SIZE];
    msg[0] = DATA_TYPE_BASE_ADV;
    msg[1] = BaseLogic::getColor();
    msg[2] = BaseLogic::getColorChangeCount();

    uint16_t score = BaseLogic::getScoreSeconds();
    msg[3] = (uint8_t)(score & 0xFF);
    msg[4] = (uint8_t)((score >> 8) & 0xFF);
    msg[5] = (uint8_t)(FIRMWARE_VERSION & 0xFF);
    msg[6] = (uint8_t)((FIRMWARE_VERSION >> 8) & 0xFF);

    sendMessage(msg, sizeof(msg));
}

// ---- GATT setup ----

void BLEServer::setupAttDb() {
    att_db_util_init();

    att_db_util_add_service_uuid16(GAP_SERVICE_UUID);
    att_db_util_add_characteristic_uuid16(
        GAP_DEVICE_NAME_UUID,
        ATT_PROPERTY_READ,
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        (uint8_t*)g_advName,
        g_advNameLen
    );

    att_db_util_add_service_uuid128(UUID_BASE_SERVICE);

    colorCharHandle = att_db_util_add_characteristic_uuid128(
        UUID_BASE_COLOR_CHAR,
        (uint16_t)(ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_DYNAMIC),
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        nullptr,
        0
    );

    rxCharHandle = att_db_util_add_characteristic_uuid128(
        UUID_BASE_RX_CHAR,
        (uint16_t)(ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE | ATT_PROPERTY_DYNAMIC),
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        nullptr,
        0
    );

    txCharHandle = att_db_util_add_characteristic_uuid128(
        UUID_BASE_TX_CHAR,
        (uint16_t)(ATT_PROPERTY_READ | ATT_PROPERTY_NOTIFY | ATT_PROPERTY_DYNAMIC),
        ATT_SECURITY_NONE,
        ATT_SECURITY_NONE,
        nullptr,
        0
    );

    att_server_init(att_db_util_get_address(), &attReadCallback, &attWriteCallback);
    att_server_register_packet_handler(&attEventCallback);

    Logger::infof(MODULE, "GATT DB setup complete (color=0x%04X, rx=0x%04X, tx=0x%04X)",
                  colorCharHandle, rxCharHandle, txCharHandle);
}

// ---- Init / Update ----

void BLEServer::init() {
    buildAdvData();
    buildScanResponseData();

    hciEventCb.callback = &hciEventCallback;
    hci_add_event_handler(&hciEventCb);

    notifyCbReg.callback = &notifyCallback;
    notifyCbReg.context = nullptr;

    Logger::info(MODULE, "Initialized (waiting for BTstack WORKING state)");

    uint8_t st = hci_get_state();
    if (st == HCI_STATE_WORKING) {
        on_bt_thread(+[](){
            if (!g_attDbBuilt) {
                setupAttDb();
                g_attDbBuilt = true;
            }
            if (!g_advStarted) {
                startAdvertising_impl();
                g_advStarted = true;
            }
        });
    }
}

static uint16_t g_handleToDisconnect = HCI_CON_HANDLE_INVALID;

void BLEServer::update() {
    uint32_t now = millis();

    if (connected && conHandle != HCI_CON_HANDLE_INVALID) {
        uint32_t elapsed = now - lastActivity;
        if (elapsed >= BASE_BLE_TIMEOUT_MS) {
            Logger::warningf(MODULE, "Connection timeout (%lu ms) - disconnecting", elapsed);

            g_handleToDisconnect = conHandle;
            on_bt_thread(+[](){
                if (g_handleToDisconnect != HCI_CON_HANDLE_INVALID) {
                    gap_disconnect(g_handleToDisconnect);
                    g_handleToDisconnect = HCI_CON_HANDLE_INVALID;
                }
            });

            connected = false;
            txNotifyEnabled = false;
            pendingMessageUpdate = false;
            conHandle = HCI_CON_HANDLE_INVALID;
            setAdvertisingConnectable(true);
            return;
        }

        if (txNotifyEnabled && (now - lastGattStatusMs >= GATT_BASE_STATUS_INTERVAL_MS)) {
            lastGattStatusMs = now;
            sendGattStatus();
        }
    }
}

// ---- HCI / ATT event handlers ----

void BLEServer::onHciEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packetType != HCI_EVENT_PACKET) return;

    const uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE: {
            const uint8_t state = btstack_event_state_get_state(packet);
            Logger::infof(MODULE, "HCI state: %d (working: %d)", state, HCI_STATE_WORKING);
            if (state == HCI_STATE_WORKING) {
                if (!g_attDbBuilt) {
                    setupAttDb();
                    g_attDbBuilt = true;
                }
                if (!g_advStarted) {
                    startAdvertising_impl();
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
                lastActivity = millis();

                Logger::infof(MODULE, "Client connected (handle=0x%04X)", conHandle);

                setAdvertisingConnectable(false);
            }
            break;
        }

        case HCI_EVENT_DISCONNECTION_COMPLETE: {
            uint16_t handle = hci_event_disconnection_complete_get_connection_handle(packet);
            if (handle != conHandle || conHandle == HCI_CON_HANDLE_INVALID) break;

            uint8_t reason = hci_event_disconnection_complete_get_reason(packet);
            Logger::warningf(MODULE, "Client disconnected: reason=0x%02X", reason);

            connected = false;
            txNotifyEnabled = false;
            pendingMessageUpdate = false;
            conHandle = HCI_CON_HANDLE_INVALID;

            setAdvertisingConnectable(true);
            break;
        }

        default:
            break;
    }
}

void BLEServer::onAttEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size) {
    if (packetType != HCI_EVENT_PACKET) return;

    const uint8_t event = hci_event_packet_get_type(packet);
    if (event == ATT_EVENT_MTU_EXCHANGE_COMPLETE) {
        const uint16_t mtu = att_event_mtu_exchange_complete_get_MTU(packet);
        Logger::infof(MODULE, "MTU exchange complete: %u", mtu);
    }
}

uint16_t BLEServer::attReadCb(uint16_t conHandle, uint16_t attributeHandle,
                               uint16_t offset, uint8_t* buffer, uint16_t bufferSize) {
    if (attributeHandle == colorCharHandle) {
        if (offset == 0 && bufferSize >= 1) {
            buffer[0] = BaseLogic::getColor();
            return 1;
        }
    }
    return 0;
}

int BLEServer::attWriteCb(uint16_t conHandle, uint16_t attributeHandle,
                           uint16_t transactionMode, uint16_t offset,
                           uint8_t* buffer, uint16_t bufferSize) {
    if (attributeHandle == colorCharHandle && bufferSize >= 1) {
        lastActivity = millis();
        uint8_t color = buffer[0];

        if (color == BASE_COLOR_NEUTRAL || color == BASE_COLOR_BLUE || color == BASE_COLOR_RED) {
            BaseLogic::setColor(color);
            Logger::infof(MODULE, "Color set via BLE: 0x%02X", color);
        } else {
            Logger::warningf(MODULE, "Invalid color value: 0x%02X", color);
        }
        return 0;
    }

    if (attributeHandle == rxCharHandle && bufferSize >= 1) {
        lastActivity = millis();
        return 0;
    }

    // CCCD write for TX notifications
    if (attributeHandle == txCharHandle + 1) {
        if (bufferSize >= 2) {
            uint16_t cccdValue = little_endian_read_16(buffer, 0);
            txNotifyEnabled = (cccdValue & 0x0001) != 0;
            lastActivity = millis();
            Logger::infof(MODULE, "TX notifications %s", txNotifyEnabled ? "enabled" : "disabled");
        }
        return 0;
    }

    return 0;
}
