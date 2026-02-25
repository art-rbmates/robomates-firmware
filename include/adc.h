#ifndef ADC_H
#define ADC_H

#include <Arduino.h>

class ADC {
    public:
        static void update();
        static uint32_t batteryMillivolts();
        static bool isBatteryLow();  // Returns true if either low-battery condition is met
    private:
        static uint32_t s_battery_mV;
        static unsigned long s_lastAboveThresholdMs;   // Last time battery was above sustained-low threshold
        static unsigned long s_lastBelowThresholdMs;   // Last time battery dropped below discharge threshold (0 = never)
};

#endif // ADC_H