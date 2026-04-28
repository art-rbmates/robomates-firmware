#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <FastLED.h>
#include <Arduino.h>
#include "config.h"

class LEDController {
public:
    static void init();
    static void setSolidColor(CRGB color);
    static void setLED(uint8_t index, CRGB color);
    static void setBrightness(uint8_t brightness);
    static void update();

    static CRGB leds[LED_NUM_LEDS];

private:
    static CRGB previous_leds[LED_NUM_LEDS];
};

#endif // LED_CONTROLLER_H
