// =============================================================================
//  MotorControl.h  -  TB6612FNG dual H-bridge driver + PID controller
//
//  Channel A -> right motor, Channel B -> left motor.
//  Signed speed: positive = forward, negative = reverse.
// =============================================================================
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Configure driver pins, take it out of standby, attach PWM. Call from setup().
void motorsInit();

// Apply signed speeds to the two motors (clamped to +/-255).
void setMotorSpeeds(int leftSpeed, int rightSpeed);

// Hard stop (both bridges low, PWM zeroed).
void stopMotors();

// PID step on the line-position error -> steering correction term.
float calculateControl(float error);

// Clear the integral + previous-error history (call when entering a turn).
void resetPID();

#endif // MOTOR_CONTROL_H
