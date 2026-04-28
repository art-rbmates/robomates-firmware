#include "base_logic.h"
#include "base_buttons.h"
#include "config.h"
#include "logger.h"
#include <Arduino.h>

static const char* MODULE = "BASE";

uint8_t BaseLogic::currentColor = BASE_COLOR_NEUTRAL;
uint8_t BaseLogic::captureTarget = BASE_COLOR_NEUTRAL;
uint32_t BaseLogic::captureStartMs = 0;
uint8_t BaseLogic::colorChangeCount = 0;
uint16_t BaseLogic::scoreSeconds = 0;
uint32_t BaseLogic::lastScoreTickMs = 0;

void BaseLogic::init() {
    currentColor = BASE_COLOR_NEUTRAL;
    captureTarget = BASE_COLOR_NEUTRAL;
    colorChangeCount = 0;
    scoreSeconds = 0;
    lastScoreTickMs = 0;
    Logger::info(MODULE, "Base logic initialized (neutral)");
}

void BaseLogic::update() {
    bool blue = BaseButtons::isBluePressed();
    bool red = BaseButtons::isRedPressed();
    uint32_t now = millis();

    // --- Capture state machine ---
    if (captureTarget != BASE_COLOR_NEUTRAL) {
        bool cancel = false;

        if (captureTarget == BASE_COLOR_BLUE) {
            if (!blue || red) cancel = true;
        } else {
            if (!red || blue) cancel = true;
        }

        if (cancel) {
            Logger::infof(MODULE, "Capture cancelled (was targeting %s)",
                          captureTarget == BASE_COLOR_BLUE ? "blue" : "red");
            captureTarget = BASE_COLOR_NEUTRAL;
        } else if ((now - captureStartMs) >= CAPTURE_TIME_MS) {
            currentColor = captureTarget;
            colorChangeCount++;
            scoreSeconds = 0;
            lastScoreTickMs = now;
            Logger::infof(MODULE, "Base captured! Now %s (change #%u)",
                          currentColor == BASE_COLOR_BLUE ? "BLUE" : "RED",
                          colorChangeCount);
            captureTarget = BASE_COLOR_NEUTRAL;
        }
        return;
    }

    // --- Start new capture ---
    if (blue && !red && currentColor != BASE_COLOR_BLUE) {
        captureTarget = BASE_COLOR_BLUE;
        captureStartMs = now;
        Logger::info(MODULE, "Capture started: BLUE");
    } else if (red && !blue && currentColor != BASE_COLOR_RED) {
        captureTarget = BASE_COLOR_RED;
        captureStartMs = now;
        Logger::info(MODULE, "Capture started: RED");
    }

    // --- Score ticking (only when captured and not contested) ---
    if (currentColor != BASE_COLOR_NEUTRAL && lastScoreTickMs > 0) {
        bool contested = blue && red;
        if (!contested && (now - lastScoreTickMs) >= 1000) {
            scoreSeconds++;
            lastScoreTickMs += 1000;
            Logger::debugf(MODULE, "Score tick: %u seconds (%s)",
                           scoreSeconds,
                           currentColor == BASE_COLOR_BLUE ? "blue" : "red");
        }
        if (contested) {
            // Both pressed - keep the tick timer moving so paused time doesn't count
            lastScoreTickMs = now;
        }
    }
}

uint8_t BaseLogic::getColor() { return currentColor; }
bool BaseLogic::isCapturing() { return captureTarget != BASE_COLOR_NEUTRAL; }
uint8_t BaseLogic::getCaptureTarget() { return captureTarget; }
uint8_t BaseLogic::getColorChangeCount() { return colorChangeCount; }
uint16_t BaseLogic::getScoreSeconds() { return scoreSeconds; }

void BaseLogic::setColor(uint8_t color) {
    captureTarget = BASE_COLOR_NEUTRAL;
    currentColor = color;
    colorChangeCount++;
    scoreSeconds = 0;
    lastScoreTickMs = (color != BASE_COLOR_NEUTRAL) ? millis() : 0;
    Logger::infof(MODULE, "Color set externally: %s (change #%u)",
                  color == BASE_COLOR_BLUE ? "BLUE" :
                  color == BASE_COLOR_RED  ? "RED" : "NEUTRAL",
                  colorChangeCount);
}
