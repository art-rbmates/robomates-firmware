#ifndef ATECC_CRYPTO_H
#define ATECC_CRYPTO_H

#include <Arduino.h>
#include <stdint.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

class ATECCCrypto {
public:
    // Initialize ATECC508A with provisioning and locking
    // Returns true on success, false on failure
    bool init();

    // Get the stored public key (must call init() first)
    // Returns pointer to 64-byte public key or nullptr if not initialized
    const uint8_t* getPublicKey() const;
    
    // Get the last byte of the public key (useful for RF addressing)
    uint8_t getPublicKeyLastByte() const;

    // Copy public key to output buffer
    // Returns true on success, false if not initialized
    bool copyPublicKey(uint8_t out64[64]) const;

    // Signs a 32-byte digest with the private key in ATECC_KEY_SLOT. Returns raw ECDSA r||s (64 bytes)
    bool signMessage32(const uint8_t msg32[32], uint8_t sig64[64]);

    // Convenience wrapper that hashes ASCII to 32 bytes using SHA-256 and then calls signMessage32
    bool signAscii(const char* ascii, uint8_t sig64[64]);

    // Helper function to sign RF data packets
    bool signRFData(const void* data, size_t dataLen, uint8_t sig64[64]);

    // Check if the crypto chip is initialized and ready
    bool isInitialized() const { return initialized; }

    // Demo function to show ATECC508A functionality
    void demo();

    // Debug logging helpers
    static void printHex(const char* label, const uint8_t* buf, size_t len, bool newlineEvery16 = true);
    void printLockStates();

    // Singleton access
    static ATECCCrypto& getInstance();

private:
    ATECCX08A atecc;
    uint8_t publicKey[64];
    bool initialized;

    // Private constructor for singleton
    ATECCCrypto() : initialized(false) {}
    
    // Disable copy constructor and assignment
    ATECCCrypto(const ATECCCrypto&) = delete;
    ATECCCrypto& operator=(const ATECCCrypto&) = delete;
};

#endif // ATECC_CRYPTO_H
