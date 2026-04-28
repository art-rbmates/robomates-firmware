#include "led_controller.h"
#include "logger.h"
#include "config.h"

static const char* MODULE = "LED";

// Static member definitions
CRGB LEDController::leds[LED_NUM_LEDS];
CRGB LEDController::previous_leds[LED_NUM_LEDS];

void LEDController::init() {
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, LED_NUM_LEDS);
    FastLED.setBrightness(50);
    
    // Initialize all LEDs to black
    for (int i = 0; i < LED_NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
        previous_leds[i] = CRGB::Black;
    }
    FastLED.show();
    
    Logger::infof(MODULE, "Initialized %d LEDs", LED_NUM_LEDS);
}

void LEDController::setEyes(CRGB color) {
    leds[LED_R_EYE] = color;
    leds[LED_L_EYE] = color;
}

CRGB LEDController::getEyeColor() {
    return leds[LED_R_EYE];
}

CRGB LEDController::getLED(LEDPositions pos) {
    return leds[pos];
}

void LEDController::setSolidColor(CRGB color) {
    for (int i = 0; i < LED_NUM_LEDS; i++) {
        leds[i] = color;
    }
}

void LEDController::setBrightness(uint8_t brightness) {
    FastLED.setBrightness(brightness);
}

void LEDController::setLED(LEDPositions led_pos, CRGB color) {
    if (led_pos < LED_NUM_LEDS) {
        leds[led_pos] = color;
    }
}

void LEDController::showLEDs() {
    FastLED.show();
}

void LEDController::update() {
    // Periodic forced refresh to correct any glitches from electrical noise (e.g. motor EMI)
    static unsigned long lastForceRefreshMs = 0;
    unsigned long now = millis();
    bool forceRefresh = (now - lastForceRefreshMs >= LED_REFRESH_INTERVAL_MS);
    
    bool leds_changed = false;
    for (int i = 0; i < LED_NUM_LEDS; i++) {
        if (leds[i] != previous_leds[i]) {
            leds_changed = true;
            break;
        }
    }
    if (leds_changed || forceRefresh) {
        for (int i = 0; i < LED_NUM_LEDS; i++) {
            previous_leds[i] = leds[i];
        }
        FastLED.show();
        if (forceRefresh) {
            lastForceRefreshMs = now;
        }
    }
}

// Shared eye colors array and index
static const CRGB eyeColors[] = {
    CRGB::Red,
    CRGB::Lime,
    CRGB::Blue,
    CRGB::Cyan,
    CRGB::Magenta,
    CRGB::Yellow
};    
static const int colorCount = sizeof(eyeColors) / sizeof(eyeColors[0]);
static int currentColorIndex = 0;

void LEDController::updateEyeColorCycling(bool leftButton, bool rightButton) {
    static bool leftButtonPrev = false;
    static bool rightButtonPrev = false;
    
    // Cycle left on rising edge
    if (leftButton && !leftButtonPrev) {
        currentColorIndex = (currentColorIndex - 1 + colorCount) % colorCount;
        setEyes(eyeColors[currentColorIndex]);
        Logger::debugf(MODULE, "Eye color cycled left to index %d", currentColorIndex);
    }
    
    // Cycle right on rising edge
    if (rightButton && !rightButtonPrev) {
        currentColorIndex = (currentColorIndex + 1) % colorCount;
        setEyes(eyeColors[currentColorIndex]);
        Logger::debugf(MODULE, "Eye color cycled right to index %d", currentColorIndex);
    }
    
    leftButtonPrev = leftButton;
    rightButtonPrev = rightButton;
}

void LEDController::cycleEyeColor() {
    currentColorIndex = (currentColorIndex + 1) % colorCount;
    setEyes(eyeColors[currentColorIndex]);
    Logger::debugf(MODULE, "Eye color cycled to index %d", currentColorIndex);
}