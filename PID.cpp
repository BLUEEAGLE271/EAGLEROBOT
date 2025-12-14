#include "PID.h"
#include <Arduino.h> // Needed for millis() and abs()

PIDController::PIDController(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
    previousError = 0;
    integral = 0;
    lastTime = millis();
}

float PIDController::compute(float setpoint, float measured) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
    
    // Safety check for divide by zero or timer wrap-around
    if (dt <= 0) return 0; 
    lastTime = currentTime;

    // 1. Error
    float error = setpoint - measured;

    // 2. Proportional
    float P = Kp * error;

    // 3. Integral
    integral += error * dt;
    // Anti-windup: clamp integral to prevent it from growing infinitely
    if (integral > 100) integral = 100;
    if (integral < -100) integral = -100;
    float I = Ki * integral;

    // 4. Derivative
    float derivative = (error - previousError) / dt;
    float D = Kd * derivative;

    previousError = error;

    // Sum and Return
    return P + I + D;
}

void PIDController::reset() {
    integral = 0;
    previousError = 0;
    lastTime = millis();
}

void PIDController::setGains(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
    reset();
}