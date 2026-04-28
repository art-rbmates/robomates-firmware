#include "main_robot.h"

#include <Arduino.h>
#include <esp_system.h>

#include "logger.h"
#include "config.h"
#include "led_controller.h"
#include "base_buttons.h"
#include "base_logic.h"
#include "ble_server.h"
#include "serial_protocol.h"

static uint32_t lastAdvUpdateMs = 0;

static void updateLEDs() {
    bool bothPressed = BaseButtons::isBluePressed() && BaseButtons::isRedPressed();

    if (bothPressed) {
        LEDController::setSolidColor(CRGB(255, 0, 255));  // Pink (blue + red)
        return;
    }

    uint8_t color = BaseLogic::getColor();
    bool capturing = BaseLogic::isCapturing();

    if (capturing) {
        uint8_t target = BaseLogic::getCaptureTarget();
        bool on = ((millis() / (CAPTURE_BLINK_PERIOD_MS / 2)) % 2) == 0;

        if (on) {
            CRGB c = (target == BASE_COLOR_BLUE) ? CRGB::Blue : CRGB::Red;
            LEDController::setSolidColor(c);
        } else {
            LEDController::setSolidColor(CRGB::Black);
        }
    } else if (color == BASE_COLOR_BLUE) {
        LEDController::setSolidColor(CRGB::Blue);
    } else if (color == BASE_COLOR_RED) {
        LEDController::setSolidColor(CRGB::Red);
    } else {
        LEDController::setSolidColor(CRGB(20, 20, 20));  // Neutral - dim white
    }
}

void robot_setup() {
    LEDController::init();
    LEDController::setSolidColor(CRGB::White);
    LEDController::update();

    Logger::init(LogLevel::INFO);
    Logger::info("MAIN", "=== Robomates Base Initialization ===");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    Logger::infof("MAIN", "BT MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    BaseButtons::init();
    BaseLogic::init();
    SerialProtocol::init();
    BLEServer::init();

    LEDController::setSolidColor(CRGB(20, 20, 20));
    LEDController::update();

    Logger::info("MAIN", "=== Base Ready ===");
}

void robot_loop() {
    SerialProtocol::update();
    BaseButtons::update();
    BaseLogic::update();
    BLEServer::update();

    updateLEDs();
    LEDController::update();

    uint32_t now = millis();
    if (now - lastAdvUpdateMs >= ADV_UPDATE_INTERVAL_MS) {
        lastAdvUpdateMs = now;
        BLEServer::updateAdvStatusData(
            BaseLogic::getColor(),
            BaseLogic::getColorChangeCount(),
            BaseLogic::getScoreSeconds()
        );
    }

    delay(10);
}
