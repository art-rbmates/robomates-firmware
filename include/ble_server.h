#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <stdint.h>
#include <stddef.h>
#include "config.h"

class BLEServer {
public:
    static void init();
    static void update();
    static void updateAdvStatusData(uint8_t baseColor, uint8_t changeCount, uint16_t scoreSeconds);
    static void rebuildAdvertising();

    static void onHciEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size);
    static void onAttEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size);
    static uint16_t attReadCb(uint16_t conHandle, uint16_t attributeHandle,
                              uint16_t offset, uint8_t* buffer, uint16_t bufferSize);
    static int attWriteCb(uint16_t conHandle, uint16_t attributeHandle,
                          uint16_t transactionMode, uint16_t offset,
                          uint8_t* buffer, uint16_t bufferSize);
    static void onNotifyPossible(void* context);

private:
    static void buildAdvData();
    static void buildScanResponseData();
    static void startAdvertising();
    static void setupAttDb();
    static void setAdvertisingConnectable(bool connectable);
    static void sendMessage(const uint8_t* data, size_t len);
    static void sendGattStatus();

    static bool connected;
    static uint16_t conHandle;
    static uint16_t colorCharHandle;
    static uint16_t rxCharHandle;
    static uint16_t txCharHandle;
    static bool txNotifyEnabled;
    static uint32_t lastActivity;

    static uint8_t messageBytes[32];
    static size_t messageLen;
    static bool pendingMessageUpdate;
};

#endif // BLE_SERVER_H
