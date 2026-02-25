#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <FastLED.h>
#include <Arduino.h>
#include "config.h"

class LEDController {
public:
    static void init();
    static void setEyes(CRGB color);
    static CRGB getEyeColor();  // Get current eye color (returns right eye)
    static void setSolidColor(CRGB color);  // Set all LEDs to same color
    static void setBrightness(uint8_t brightness);
    static void setLED(LEDPositions led_pos, CRGB color);
    static CRGB getLED(LEDPositions pos);  // Get current color of a specific LED
    static void showLEDs();
    static void update();
    
    // Handle eye color cycling with controller left/right buttons
    static void updateEyeColorCycling(bool leftButton, bool rightButton);
    
    // Cycle to next eye color (for single-button use)
    static void cycleEyeColor();

private:
    static CRGB leds[LED_NUM_LEDS];
    static CRGB previous_leds[LED_NUM_LEDS];
};

#endif // LED_CONTROLLER_H
