#include "logger.h"
#include <stdarg.h>

// Static member initialization
LogLevel Logger::currentLevel = LogLevel::INFO;
SemaphoreHandle_t Logger::mutex = nullptr;

void Logger::init(LogLevel level) {
    currentLevel = level;
    if (mutex == nullptr) {
        mutex = xSemaphoreCreateMutex();
    }
    Serial.begin(115200);
    delay(100);
}

void Logger::setLevel(LogLevel level) {
    currentLevel = level;
}

LogLevel Logger::getLevel() {
    return currentLevel;
}

const char* Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG:    return "DEBUG";
        case LogLevel::INFO:     return "INFO";
        case LogLevel::WARNING:  return "WARN";
        case LogLevel::ERROR:    return "ERROR";
        case LogLevel::CRITICAL: return "CRIT";
        default:                 return "UNKNOWN";
    }
}

const char* Logger::levelToEmoji(LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG:    return "🔍";
        case LogLevel::INFO:     return "ℹ️";
        case LogLevel::WARNING:  return "⚠️";
        case LogLevel::ERROR:    return "❌";
        case LogLevel::CRITICAL: return "🔥";
        default:                 return "❓";
    }
}

void Logger::log(LogLevel level, const char* module, const char* message) {
    if (level < currentLevel) return;
    
    unsigned long timestamp = millis();
    if (mutex == nullptr) {
        // Before init() — single-core boot, safe to print directly
        Serial.printf("[%7lu] %s [%-8s] [%s] %s\n", 
                      timestamp,
                      levelToEmoji(level),
                      levelToString(level),
                      module,
                      message);
    } else if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.printf("[%7lu] %s [%-8s] [%s] %s\n", 
                      timestamp,
                      levelToEmoji(level),
                      levelToString(level),
                      module,
                      message);
        xSemaphoreGive(mutex);
    }
    // If mutex exists but can't be taken within 50ms, silently drop the log to avoid deadlock
}

void Logger::logf(LogLevel level, const char* module, const char* format, va_list args) {
    if (level < currentLevel) return;
    
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    log(level, module, buffer);
}

void Logger::debug(const char* module, const char* message) {
    log(LogLevel::DEBUG, module, message);
}

void Logger::info(const char* module, const char* message) {
    log(LogLevel::INFO, module, message);
}

void Logger::warning(const char* module, const char* message) {
    log(LogLevel::WARNING, module, message);
}

void Logger::error(const char* module, const char* message) {
    log(LogLevel::ERROR, module, message);
}

void Logger::critical(const char* module, const char* message) {
    log(LogLevel::CRITICAL, module, message);
}

void Logger::debugf(const char* module, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logf(LogLevel::DEBUG, module, format, args);
    va_end(args);
}

void Logger::infof(const char* module, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logf(LogLevel::INFO, module, format, args);
    va_end(args);
}

void Logger::warningf(const char* module, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logf(LogLevel::WARNING, module, format, args);
    va_end(args);
}

void Logger::errorf(const char* module, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logf(LogLevel::ERROR, module, format, args);
    va_end(args);
}

void Logger::criticalf(const char* module, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logf(LogLevel::CRITICAL, module, format, args);
    va_end(args);
}

