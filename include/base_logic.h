#ifndef BASE_LOGIC_H
#define BASE_LOGIC_H

#include <stdint.h>

class BaseLogic {
public:
    static void init();
    static void update();

    static uint8_t getColor();
    static bool isCapturing();
    static uint8_t getCaptureTarget();

    static uint8_t getColorChangeCount();
    static uint16_t getScoreSeconds();
    static void setColor(uint8_t color);

private:
    static uint8_t currentColor;
    static uint8_t captureTarget;
    static uint32_t captureStartMs;

    static uint8_t colorChangeCount;
    static uint16_t scoreSeconds;
    static uint32_t lastScoreTickMs;
};

#endif // BASE_LOGIC_H
