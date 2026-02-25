#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Log levels
enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4,
    NONE = 5  // Disable all logging
};

class Logger {
public:
    // Initialize the logger
    static void init(LogLevel level = LogLevel::INFO);
    
    // Set log level
    static void setLevel(LogLevel level);
    
    // Get current log level
    static LogLevel getLevel();
    
    // Log methods
    static void debug(const char* module, const char* message);
    static void info(const char* module, const char* message);
    static void warning(const char* module, const char* message);
    static void error(const char* module, const char* message);
    static void critical(const char* module, const char* message);
    
    // Formatted log methods
    static void debugf(const char* module, const char* format, ...);
    static void infof(const char* module, const char* format, ...);
    static void warningf(const char* module, const char* format, ...);
    static void errorf(const char* module, const char* format, ...);
    static void criticalf(const char* module, const char* format, ...);
    
private:
    static LogLevel currentLevel;
    static SemaphoreHandle_t mutex;
    static void log(LogLevel level, const char* module, const char* message);
    static void logf(LogLevel level, const char* module, const char* format, va_list args);
    static const char* levelToString(LogLevel level);
    static const char* levelToEmoji(LogLevel level);
};

#endif // LOGGER_H

