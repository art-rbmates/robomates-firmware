#include "base_buttons.h"
#include "config.h"
#include "logger.h"
#include <Arduino.h>

static const char* MODULE = "BTN";

bool BaseButtons::bluePressed = false;
bool BaseButtons::redPressed = false;
uint32_t BaseButtons::blueLastChangeMs = 0;
uint32_t BaseButtons::redLastChangeMs = 0;
bool BaseButtons::blueRaw = false;
bool BaseButtons::redRaw = false;

void BaseButtons::init() {
    pinMode(BUTTON_BLUE_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RED_PIN, INPUT_PULLUP);
    Logger::infof(MODULE, "Buttons initialized (blue=GPIO%d, red=GPIO%d, INPUT_PULLUP)",
                  BUTTON_BLUE_PIN, BUTTON_RED_PIN);
}

void BaseButtons::update() {
    uint32_t now = millis();

    // INPUT_PULLUP: pressed = LOW
    bool newBlueRaw = (digitalRead(BUTTON_BLUE_PIN) == LOW);
    bool newRedRaw = (digitalRead(BUTTON_RED_PIN) == LOW);

    if (newBlueRaw != blueRaw) {
        blueRaw = newBlueRaw;
        blueLastChangeMs = now;
    }
    if (newRedRaw != redRaw) {
        redRaw = newRedRaw;
        redLastChangeMs = now;
    }

    if ((now - blueLastChangeMs) >= BUTTON_DEBOUNCE_MS) {
        if (bluePressed != blueRaw) {
            bluePressed = blueRaw;
            Logger::debugf(MODULE, "Blue button %s", bluePressed ? "PRESSED" : "released");
        }
    }
    if ((now - redLastChangeMs) >= BUTTON_DEBOUNCE_MS) {
        if (redPressed != redRaw) {
            redPressed = redRaw;
            Logger::debugf(MODULE, "Red button %s", redPressed ? "PRESSED" : "released");
        }
    }
}

bool BaseButtons::isBluePressed() { return bluePressed; }
bool BaseButtons::isRedPressed() { return redPressed; }
