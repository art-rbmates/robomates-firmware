#include "led_controller.h"
#include "logger.h"
#include "config.h"

static const char* MODULE = "LED";

CRGB LEDController::leds[LED_NUM_LEDS];
CRGB LEDController::previous_leds[LED_NUM_LEDS];

void LEDController::init() {
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, LED_NUM_LEDS);
    FastLED.setBrightness(50);

    for (int i = 0; i < LED_NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
        previous_leds[i] = CRGB::Black;
    }
    FastLED.show();

    Logger::infof(MODULE, "Initialized %d LEDs on pin %d", LED_NUM_LEDS, LED_DATA_PIN);
}

void LEDController::setSolidColor(CRGB color) {
    for (int i = 0; i < LED_NUM_LEDS; i++) {
        leds[i] = color;
    }
}

void LEDController::setLED(uint8_t index, CRGB color) {
    if (index < LED_NUM_LEDS) {
        leds[index] = color;
    }
}

void LEDController::setBrightness(uint8_t brightness) {
    FastLED.setBrightness(brightness);
}

void LEDController::update() {
    static unsigned long lastForceRefreshMs = 0;
    unsigned long now = millis();
    bool forceRefresh = (now - lastForceRefreshMs >= 3000);

    bool changed = false;
    for (int i = 0; i < LED_NUM_LEDS; i++) {
        if (leds[i] != previous_leds[i]) {
            changed = true;
            break;
        }
    }

    if (changed || forceRefresh) {
        for (int i = 0; i < LED_NUM_LEDS; i++) {
            previous_leds[i] = leds[i];
        }
        FastLED.show();
        if (forceRefresh) lastForceRefreshMs = now;
    }
}
