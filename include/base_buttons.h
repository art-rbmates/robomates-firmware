#ifndef BASE_BUTTONS_H
#define BASE_BUTTONS_H

#include <stdint.h>

class BaseButtons {
public:
    static void init();
    static void update();

    static bool isBluePressed();
    static bool isRedPressed();

private:
    static bool bluePressed;
    static bool redPressed;
    static uint32_t blueLastChangeMs;
    static uint32_t redLastChangeMs;
    static bool blueRaw;
    static bool redRaw;
};

#endif // BASE_BUTTONS_H
