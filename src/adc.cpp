#include "adc.h"
#include "config.h"

uint32_t ADC::s_battery_mV = 0;
unsigned long ADC::s_lastAboveThresholdMs = 0;
unsigned long ADC::s_lastBelowThresholdMs = 0;

uint32_t ADC::batteryMillivolts() {
    return s_battery_mV;
}

bool ADC::isBatteryLow() {
    unsigned long now = millis();

    // Condition 1: voltage has been continuously below BATTERY_LOW_WARNING_THRESHOLD_MV for BATTERY_LOW_WARNING_TIMEOUT_MS
    bool sustainedLow = (now - s_lastAboveThresholdMs) >= BATTERY_LOW_WARNING_TIMEOUT_MS;

    // Condition 2: voltage dropped below BATTERY_DISCHARGE_WARNING_THRESHOLD_MV at least once within BATTERY_DISCHARGE_WARNING_WINDOW_MS
    bool dischargeSpike = (s_lastBelowThresholdMs != 0) &&
                          ((now - s_lastBelowThresholdMs) < BATTERY_DISCHARGE_WARNING_WINDOW_MS);

    return sustainedLow || dischargeSpike;
}

void ADC::update() {
    static unsigned long lastBatteryRead = 0;
    static bool firstReading = true;
    unsigned long now = millis();
    
    if (now - lastBatteryRead >= BATTERY_READ_INTERVAL_MS) {
        long sum = 0;
        const long N = 16;
        for (int i = 0; i < N; i++) {
            sum += analogReadMilliVolts(BATTERY_ADC_PIN);
        }
        // 2:1 divider → battery ≈ 2x the pin voltage
        s_battery_mV = sum * VOLTAGE_DIVIDER_RATIO / N;
        lastBatteryRead = now;
        
        // Initialize timestamp on first reading to avoid immediate warning on boot
        if (firstReading) {
            s_lastAboveThresholdMs = now;
            firstReading = false;
        }
        
        // Condition 1 tracking: last time battery was above sustained-low threshold
        if (s_battery_mV >= BATTERY_LOW_WARNING_THRESHOLD_MV) {
            s_lastAboveThresholdMs = now;
        }
        
        // Condition 2 tracking: last time battery dropped below discharge threshold
        if (s_battery_mV < BATTERY_DISCHARGE_WARNING_THRESHOLD_MV) {
            s_lastBelowThresholdMs = now;
        }
    }
}
