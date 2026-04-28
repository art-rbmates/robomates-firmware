#include "scan.h"
#include "logger.h"
#include "led_controller.h"
#include "balance_controller.h"
#include <Arduino.h>
#include "config.h"

static const char* MODULE = "Scan";

// Static member initialization
bool Scan::scanBurstActive = false;
unsigned long Scan::scanBurstStartTime = 0;
unsigned long Scan::scanCooldownEndTime = 0;
bool Scan::r1ButtonLastState = false;
bool Scan::l1ButtonLastState = false;
bool Scan::shouldSendScanFlag = false;
unsigned long Scan::lastScanSentTime = 0;
bool Scan::rumbleNeeded = false;

void Scan::update(bool r1_current_state, bool l1_current_state, bool &shouldRumble, bool inCrawlMode) {
    // Detect R1 or L1 button press (rising edge) and check if not in cooldown
    bool r1_pressed = r1_current_state && !r1ButtonLastState;
    bool l1_pressed = l1_current_state && !l1ButtonLastState;
    unsigned long now = millis();

    if (now >= scanCooldownEndTime && rumbleNeeded) {
        shouldRumble = true;
        rumbleNeeded = false;
    } else {
        shouldRumble = false;
    }
    
    // Don't allow scan activation in crawl mode or when motors are disabled
    // Also cancel any active scan burst and turn off LEDs
    if (inCrawlMode || !BalanceController::areMotorsEnabled()) {
        if (scanBurstActive) {
            scanBurstActive = false;
            LEDController::setLED(LED_L_FTOP, CRGB(0, 0, 0));
            LEDController::setLED(LED_L_FBOT, CRGB(0, 0, 0));
            LEDController::setLED(LED_L_BACK, CRGB(0, 0, 0));
            LEDController::setLED(LED_R_FTOP, CRGB(0, 0, 0));
            LEDController::setLED(LED_R_FBOT, CRGB(0, 0, 0));
            LEDController::setLED(LED_R_BACK, CRGB(0, 0, 0));
        }
        r1ButtonLastState = r1_current_state;
        l1ButtonLastState = l1_current_state;
        return;
    }
        
    if ((r1_pressed || l1_pressed) && !scanBurstActive && now >= scanCooldownEndTime) {
        scanBurstActive = true;
        scanBurstStartTime = now;
        scanCooldownEndTime = now + SCAN_BURST_DURATION + SCAN_COOLDOWN_DURATION;
        rumbleNeeded = true;
    }
    
    if (scanBurstActive) {
        if (now - scanBurstStartTime >= SCAN_BURST_DURATION) {
            // Burst finished - set all LEDs to black
            scanBurstActive = false;
            LEDController::setLED(LED_L_FTOP, CRGB(0, 0, 0));
            LEDController::setLED(LED_L_FBOT, CRGB(0, 0, 0));
            LEDController::setLED(LED_L_BACK, CRGB(0, 0, 0));
            LEDController::setLED(LED_R_FTOP, CRGB(0, 0, 0));
            LEDController::setLED(LED_R_FBOT, CRGB(0, 0, 0));
            LEDController::setLED(LED_R_BACK, CRGB(0, 0, 0));
        } else {
            // Burst active - continuously set LEDs to magenta (in case something overwrites them)
            LEDController::setLED(LED_L_FTOP, CRGB(255, 0, 255));
            LEDController::setLED(LED_L_FBOT, CRGB(255, 0, 255));
            LEDController::setLED(LED_L_BACK, CRGB(255, 0, 255));
            LEDController::setLED(LED_R_FTOP, CRGB(255, 0, 255));
            LEDController::setLED(LED_R_FBOT, CRGB(255, 0, 255));
            LEDController::setLED(LED_R_BACK, CRGB(255, 0, 255));
            
            // Send scan periodically during burst
            if (now - lastScanSentTime >= SCAN_SEND_INTERVAL) {
                shouldSendScanFlag = true;
            }
        }
    }    
    
    r1ButtonLastState = r1_current_state;
    l1ButtonLastState = l1_current_state;
}

bool Scan::shouldSendScan() {
    if (shouldSendScanFlag) {
        shouldSendScanFlag = false;
        lastScanSentTime = millis();
        return true;
    }
    return false;
}
