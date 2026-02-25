#ifndef BLE_SERVER_H
#define BLE_SERVER_H

#include <stdint.h>
#include <stddef.h>
#include "config.h"
#include "cc1101.h"

class BLEServer {
public:
    static void init();
    static bool isConnected();
    
    // Send messages to central (new protocol)
    static void sendPingToCentral(const MyCC1101::PingMessage& msg);
    
    static const uint8_t* getLastRx(size_t& len);
    static void update();
    
    static void stopAdvertising();
    static void resumeAdvertising();
    static void rebuildAdvertising();  // Rebuild adv data and restart (for name changes)
    
    // These need to be public so C-style callback wrappers can access them
    static void onHciEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size);
    static void onAttEvent(uint8_t packetType, uint16_t channel, uint8_t* packet, uint16_t size);
    static uint16_t attReadCb(uint16_t conHandle, uint16_t attributeHandle, uint16_t offset, 
                            uint8_t* buffer, uint16_t bufferSize);
    static int attWriteCb(uint16_t conHandle, uint16_t attributeHandle, uint16_t transactionMode,
                        uint16_t offset, uint8_t* buffer, uint16_t bufferSize);
    static void onNotifyPossible(void* context);
    
private:
    
    static bool connected;
    static uint16_t conHandle;
    static uint16_t txValueHandle;
    static uint16_t rxValueHandle;
    static bool txNotifyEnabled;
    
    // Message buffer (sized for largest message)
    static uint8_t messageBytes[MSG_SIZE_MAX];
    static size_t messageLen;
    static bool pendingMessageUpdate;
    
    // Last RX data
    static uint8_t lastRx[256];
    static size_t lastRxLen;
    
    // Internal helper functions
    static void startAdvertising();
    static void buildAdvData();
    static void setupAttDb();
    static void logHex(const char* prefix, const uint8_t* data, size_t len);
    static void sendMessage(const uint8_t* data, size_t len);
    
    // Subprogram BLE handlers
    static void handleListSubprograms(uint8_t startIndex);
    static void handleCreateSubprogram(const uint8_t* data, size_t len);
    static void handleRenameSubprogram(const uint8_t* data, size_t len);
    static void handleUpdateSubprogram(const uint8_t* data, size_t len);
    static void handleSetSubprogramBtn(uint8_t buttonNum, uint8_t id);
    static void handleDeleteSubprogram(uint8_t id);
    static void handleGetSubprogram(uint8_t id);
    
    // Pitch constant BLE handlers
    static void handleGetPitchConstant();
    static void handleSetPitchConstant(const uint8_t* data, size_t len);
    
    // Steering sensitivity BLE handlers
    static void handleGetSteeringSensitivity();
    static void handleSetSteeringSensitivity(const uint8_t* data, size_t len);
    
    // Velocity limits BLE handlers
    static void handleGetVelocityLimits();
    static void handleSetVelocityLimits(const uint8_t* data, size_t len);
    
    // Calibration BLE handler
    static void handleRecalibrate();
    
    // Chunked subprogram data handlers
    static void handleReadSubprogramData(const uint8_t* data, size_t len);
    static void handleStartSubprogramWrite(const uint8_t* data, size_t len);
    static void handleWriteSubprogramData(const uint8_t* data, size_t len);
    static void handleFinishSubprogramWrite(uint8_t id);
    
    // Controller mapping handlers
    static void handleGetControllerMapping(uint8_t controllerType);
    static void handleSetControllerMapping(const uint8_t* data, size_t len);
    static void handleResetControllerMapping(uint8_t controllerType);
    
    // BLE controller input handler
    static void handleControllerInput(const uint8_t* data, size_t len);
    
    // Controller input streaming
    static void handleEnableControllerStream();
    static void sendControllerStreamData();
    static uint32_t controllerStreamEnabledAt;  // Timestamp when streaming was enabled
    static uint32_t lastControllerStreamSent;   // Timestamp of last stream packet sent
    
    // Melody BLE handlers
    static void handleReadMelodyData(const uint8_t* data, size_t len);
    static void handleStartMelodyWrite(const uint8_t* data, size_t len);
    static void handleWriteMelodyData(const uint8_t* data, size_t len);
    static void handleFinishMelodyWrite();
    static void handleResetMelody();
    static void handleGetMelodyAmplitude();
    static void handleSetMelodyAmplitude(const uint8_t* data, size_t len);
    
    // Crypto BLE handlers
    static void handleGetPublicKey();
    static void handleSignMessage(const uint8_t* data, size_t len);
};

#endif // BLE_SERVER_H

