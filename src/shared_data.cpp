#include "shared_data.h"
#include "logger.h"

SemaphoreHandle_t SharedData::mutex = nullptr;
SharedData::RobotInputs SharedData::robotInputs = {};
SharedData::MotorState SharedData::motorState = {};
SharedData::SystemState SharedData::systemState = {};
SharedData::SubprogramControl SharedData::subprogramControl = {};

void SharedData::init() {
    mutex = xSemaphoreCreateMutex();
    if (mutex == nullptr) {
        Logger::critical("SharedData", "Failed to create mutex!");
    } else {
        Logger::info("SharedData", "Mutex initialized successfully");
    }
    
    systemState.speedCoefficient = 1.0f;
    systemState.torqueCoefficient = 1.0f;
}

void SharedData::setRobotInputs(const RobotInputs& inputs) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        robotInputs = inputs;
        xSemaphoreGive(mutex);
    }
}

SharedData::RobotInputs SharedData::getRobotInputs() {
    RobotInputs inputs = {};
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        inputs = robotInputs;
        xSemaphoreGive(mutex);
    }
    return inputs;
}

void SharedData::setMotorState(const MotorState& state) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motorState = state;
        xSemaphoreGive(mutex);
    }
}

SharedData::MotorState SharedData::getMotorState() {
    MotorState state = {};
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = motorState;
        xSemaphoreGive(mutex);
    }
    return state;
}

void SharedData::setSystemState(const SystemState& state) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        systemState = state;
        xSemaphoreGive(mutex);
    }
}

SharedData::SystemState SharedData::getSystemState() {
    SystemState state = {};
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = systemState;
        xSemaphoreGive(mutex);
    }
    return state;
}

void SharedData::setSubprogramControl(const SubprogramControl& control) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        subprogramControl = control;
        xSemaphoreGive(mutex);
    }
}

SharedData::SubprogramControl SharedData::getSubprogramControl() {
    SubprogramControl control = {};
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        control = subprogramControl;
        xSemaphoreGive(mutex);
    }
    return control;
}

