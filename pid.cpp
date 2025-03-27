#include "pid.h"
#include <Arduino.h>

PidController::PidController(float p, float i, float d, int interval) :
    kp(p), ki(i), kd(d), 
    errorSum(0), lastError(0), lastTime(0),
    targetValue(0), initialValue(0), 
    isInitialized(false), isEnabled(false),
    updateInterval(interval), errorSumLimit(20.0) {
}

void PidController::setTunings(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void PidController::setInterval(int intervalMs) {
    updateInterval = intervalMs;
}

void PidController::setErrorSumLimits(float limit) {
    errorSumLimit = limit;
}

void PidController::calibrateInitialValue(float currentValue) {
    if (currentValue != 0) {
        initialValue = currentValue;
        isInitialized = true;
        targetValue = 0; // Default target is to maintain current height
        
        // Reset PID variables
        reset();
        
        Serial.print("Initial value calibrated to: ");
        Serial.print(initialValue);
        Serial.println(" m");
    } else {
        Serial.println("Cannot calibrate: Invalid sensor data");
    }
}

void PidController::setTargetValue(float newTarget) {
    targetValue = newTarget;
    // Reset integral term to avoid sudden changes
    errorSum = 0;
    
    Serial.print("Target value set to: ");
    Serial.print(targetValue);
    Serial.println(" m (relative to initial height)");
}

void PidController::enable() {
    isEnabled = true;
    lastTime = millis();
    Serial.println("PID controller enabled");
}

void PidController::disable() {
    isEnabled = false;
    Serial.println("PID controller disabled");
}

bool PidController::isActive() const {
    return isEnabled && isInitialized;
}

void PidController::reset() {
    errorSum = 0;
    lastError = 0;
    lastTime = millis();
}

int PidController::compute(float currentValue) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    
    // Prevent instability due to very small time intervals
    if (deltaTime < 0.01) {
        return 0;
    }
    
    // Calculate relative value (current value minus initial reference value)
    float relativeValue = currentValue - initialValue;
    
    // Calculate error
    float error = targetValue - relativeValue;
    
    // Integral term (with limits to prevent integral windup)
    errorSum += error * deltaTime;
    errorSum = constrain(errorSum, -errorSumLimit, errorSumLimit);
    
    // Derivative term
    float dError = (error - lastError) / deltaTime;
    
    // Calculate PID output
    float output = kp * error + ki * errorSum + kd * dError;
    
    // Update state variables for next calculation
    lastError = error;
    lastTime = currentTime;
    
    // Limit output to motor control range (-100 to 100)
    int motorOutput = constrain(output, -100, 100);
    
    // Debug output
    printDebug(relativeValue, motorOutput);
    
    return motorOutput;
}

float PidController::getTarget() const {
    return targetValue;
}

float PidController::getInitialValue() const {
    return initialValue;
}

bool PidController::isInitialValueSet() const {
    return isInitialized;
}

float PidController::getCurrentRelative(float currentValue) const {
    return currentValue - initialValue;
}

void PidController::printDebug(float currentValue, int output) {
    Serial.print("PID: Target=");
    Serial.print(targetValue);
    Serial.print("m, Current=");
    Serial.print(currentValue);
    Serial.print("m, Error=");
    Serial.print(targetValue - currentValue);
    Serial.print(", Output=");
    Serial.println(output);
}