#include "atecc_crypto.h"
#include "logger.h"
#include "config.h"
#include <Wire.h>
#include "mbedtls/sha256.h"
#include "shared_i2c.h"
#include <string.h>

static const char* MODULE = "ATECC";

// Singleton instance
ATECCCrypto& ATECCCrypto::getInstance() {
  static ATECCCrypto instance;
  return instance;
}

bool ATECCCrypto::init() {
  Logger::info(MODULE, "=== ATECC508A Initialization ===");
  if (!SharedI2C::isInitialized()) {
    Logger::warning(MODULE, "Shared I2C not initialized, initializing now...");
    if (!SharedI2C::init()) {
      Logger::error(MODULE, "Failed to initialize shared I2C");
      return false;
    }
  }
  
  TwoWire& i2cBus = SharedI2C::getBus();
  if (!atecc.begin(ATECC_I2C_ADDR, i2cBus)) {
    Logger::errorf(MODULE, "ATECC508A init failed at address 0x%02X", ATECC_I2C_ADDR);
    return false;
  }
  Logger::info(MODULE, "ATECC508A initialized successfully");
  
  if (!atecc.readConfigZone(false)) {
    Logger::error(MODULE, "Failed to read config zone");
    return false;
  }
  
  Logger::info(MODULE, "Device Serial Number:");
  printHex("", atecc.serialNumber, 9, false);
  
  printLockStates();
  
  // **IDEMPOTENT PROVISIONING - IRREVERSIBLE OPERATIONS**
  Logger::info(MODULE, "--- Provisioning Phase (IRREVERSIBLE) ---");
  
  // Step 1: Config zone provisioning
  if (!atecc.configLockStatus) {
    Logger::warning(MODULE, "Config zone NOT locked - writing config and locking...");
    
    if (!atecc.writeConfigSparkFun()) {
      Logger::error(MODULE, "Failed to write SparkFun config");
      return false;
    }
    Logger::info(MODULE, "SparkFun config written");
    
    if (!atecc.lockConfig()) {
      Logger::error(MODULE, "Failed to lock config zone");
      return false;
    }
    Logger::info(MODULE, "Config zone LOCKED (permanent)");
    
    // Re-read status after config lock
    if (!atecc.readConfigZone(false)) {
      Logger::error(MODULE, "Failed to re-read config after locking");
      return false;
    }
  } else {
    Logger::info(MODULE, "Config zone already locked");
  }
  
  // Step 2: Key pair generation (only if needed)
  Logger::infof(MODULE, "Checking for existing key in slot %d...", ATECC_KEY_SLOT);
  
  bool keyExists = false;
  if (atecc.generatePublicKey(ATECC_KEY_SLOT)) {
    Logger::info(MODULE, "Private key already exists in slot");
    keyExists = true;
  } else {
    Logger::info(MODULE, "No key found - generating new key pair...");
    
    if (!atecc.createNewKeyPair(ATECC_KEY_SLOT)) {
      Logger::errorf(MODULE, "Failed to create key pair in slot %d", ATECC_KEY_SLOT);
      return false;
    }
    Logger::infof(MODULE, "New key pair created in slot %d", ATECC_KEY_SLOT);
    keyExists = true;
  }
  
  // Step 3: Data/OTP locking
  if (!atecc.dataOTPLockStatus) {
    Logger::warning(MODULE, "Data/OTP zone NOT locked - locking...");
    
    if (!atecc.lockDataAndOTP()) {
      Logger::error(MODULE, "Failed to lock data/OTP zone");
      return false;
    }
    Logger::info(MODULE, "Data/OTP zone LOCKED (permanent)");
  } else {
    Serial.println("✓ Data/OTP zone already locked");
  }
  
  // Step 4: Slot 0 locking
  if (!atecc.slot0LockStatus) {
    Serial.println("⚠️  Slot 0 NOT locked - locking...");
    
    if (!atecc.lockDataSlot0()) {
      Serial.println("ERROR: Failed to lock slot 0");
      return false;
    }
    Serial.println("🔒 Slot 0 LOCKED (permanent)");
  } else {
    Serial.println("✓ Slot 0 already locked");
  }
  
  // Final status check
  if (!atecc.readConfigZone(false)) {
    Serial.println("ERROR: Failed to read final config status");
    return false;
  }
  
  Serial.println("\n--- Final Status ---");
  printLockStates();
  
  if (atecc.configLockStatus && atecc.dataOTPLockStatus && atecc.slot0LockStatus && keyExists) {
    // Store the public key in our class member
    if (atecc.generatePublicKey(ATECC_KEY_SLOT)) {
      memcpy(publicKey, atecc.publicKey64Bytes, 64);
      initialized = true;
      Serial.println("✅ ATECC508A fully provisioned and ready");
      Serial.print("Public key stored, last byte: 0x");
      Serial.println(publicKey[63], HEX);
      return true;
    } else {
      Serial.println("ERROR: Failed to retrieve public key for storage");
      return false;
    }
  } else {
    Serial.println("❌ ATECC508A provisioning incomplete");
    return false;
  }
}

const uint8_t* ATECCCrypto::getPublicKey() const {
  if (!initialized) {
    return nullptr;
  }
  return publicKey;
}

uint8_t ATECCCrypto::getPublicKeyLastByte() const {
  if (!initialized) {
    return 0;
  }
  return publicKey[63];
}

bool ATECCCrypto::copyPublicKey(uint8_t out64[64]) const {
  if (!initialized) {
    return false;
  }
  memcpy(out64, publicKey, 64);
  return true;
}

bool ATECCCrypto::signMessage32(const uint8_t msg32[32], uint8_t sig64[64]) {
  if (!initialized) {
    Logger::error(MODULE, "ATECC not initialized");
    return false;
  }
  
  Logger::info(MODULE, "Starting signature operation...");
  
  // Lock the shared I2C bus to prevent motor sensor from accessing it
  SharedI2C::lock();
  Logger::info(MODULE, "I2C bus locked");
  
  bool success = false;
  
  do {
    // Re-establish I2C connection
    TwoWire& i2cBus = SharedI2C::getBus();
    if (!atecc.begin(ATECC_I2C_ADDR, i2cBus)) {
      Logger::error(MODULE, "Failed to re-init ATECC I2C connection");
      break;
    }
    Logger::info(MODULE, "ATECC I2C connection re-established");
    
    Logger::infof(MODULE, "Creating signature with slot %d...", ATECC_KEY_SLOT);
    if (!atecc.createSignature((uint8_t*)msg32, ATECC_KEY_SLOT)) {
      Logger::errorf(MODULE, "createSignature failed with slot %d", ATECC_KEY_SLOT);
      break;
    }
    
    Logger::info(MODULE, "Signature created successfully");
    
    // Copy the 64-byte ECDSA signature (r||s)
    memcpy(sig64, atecc.signature, 64);
    success = true;
  } while (false);
  
  SharedI2C::unlock();
  Logger::info(MODULE, "I2C bus unlocked");
  return success;
}

bool ATECCCrypto::signAscii(const char* ascii, uint8_t sig64[64]) {
  if (!initialized) {
    Logger::error(MODULE, "ATECC not initialized");
    return false;
  }
  
  uint8_t digest[32];
  
  // Hash the ASCII string with SHA-256
  int ret = mbedtls_sha256_ret((const unsigned char*)ascii, strlen(ascii), digest, 0);
  if (ret != 0) {
    Logger::errorf(MODULE, "SHA-256 failed with code %d", ret);
    return false;
  }
  
  // Sign the digest
  return signMessage32(digest, sig64);
}

bool ATECCCrypto::signRFData(const void* data, size_t dataLen, uint8_t sig64[64]) {
  if (!initialized) {
    Logger::error(MODULE, "ATECC not initialized");
    return false;
  }
  
  Logger::infof(MODULE, "signRFData called with %u bytes", dataLen);
  
  uint8_t digest[32];
  
  // Hash the data with SHA-256
  Logger::info(MODULE, "Computing SHA-256...");
  int ret = mbedtls_sha256_ret((const unsigned char*)data, dataLen, digest, 0);
  if (ret != 0) {
    Logger::errorf(MODULE, "SHA-256 failed with code %d", ret);
    return false;
  }
  Logger::info(MODULE, "SHA-256 completed");
  
  // Sign the digest
  return signMessage32(digest, sig64);
}

void ATECCCrypto::demo() {
  Serial.println("\n=== ATECC508A Demo ===");
  
  if (!initialized) {
    Serial.println("❌ ATECC not initialized - cannot run demo");
    return;
  }
  
  Serial.print("Device Serial: ");
  printHex("SN", atecc.serialNumber, 9, false);
  Serial.println();
  
  printLockStates();
  
  Serial.println();
  printHex("Public Key (X||Y)", publicKey, 64, true);
  Serial.println();
  
  // Demo message signing
  const char* msg = "Hello! I'm Robomate!";
  Serial.printf("\nMessage: \"%s\"\n", msg);
  
  // Calculate SHA-256 of the message
  uint8_t digest[32];
  int ret = mbedtls_sha256_ret((const unsigned char*)msg, strlen(msg), digest, 0);
  if (ret != 0) {
    Serial.printf("❌ SHA-256 failed with code %d\n", ret);
    return;
  }
  
  printHex("SHA-256 Digest", digest, 32, true);
  Serial.println();
  
  // Sign the message
  uint8_t sig[64];
  if (!signAscii(msg, sig)) {
    Serial.println("❌ Failed to sign message");
    return;
  }
  
  printHex("ECDSA Signature (r||s)", sig, 64, true);
  Serial.println();
  
  Serial.println("✅ Demo completed successfully");
}

void ATECCCrypto::printHex(const char* label, const uint8_t* buf, size_t len, bool newlineEvery16) {
  Serial.print(label);
  if (strlen(label) > 0) Serial.print(": ");
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    if (newlineEvery16 && ((i + 1) % 16 == 0) && (i + 1 < len)) {
      Serial.println();
      // Add spacing for label alignment
      for (size_t j = 0; j < strlen(label) + 2; j++) Serial.print(" ");
    } else if (i + 1 < len) {
      Serial.print(" ");
    }
  }
}

void ATECCCrypto::printLockStates() {
  Serial.printf("Config Lock: %s\n", atecc.configLockStatus ? "🔒 LOCKED" : "🔓 UNLOCKED");
  Serial.printf("Data/OTP Lock: %s\n", atecc.dataOTPLockStatus ? "🔒 LOCKED" : "🔓 UNLOCKED");
  Serial.printf("Slot 0 Lock: %s\n", atecc.slot0LockStatus ? "🔒 LOCKED" : "🔓 UNLOCKED");
}