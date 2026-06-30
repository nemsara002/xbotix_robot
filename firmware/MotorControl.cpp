// =============================================================================
//  MotorControl.cpp
// =============================================================================
#include "MotorControl.h"
#include "config.h"

// PID memory is private to this module.
static float previousError = 0.0f;
static float integral      = 0.0f;

void motorsInit() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);          // enable the driver

  // Arduino-ESP32 v3.x: attach PWM straight to the pin.
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
}

float calculateControl(float error) {
  float P = KP * error;

  integral += error * 0.25f;
  integral  = constrain(integral, -15.0f, 15.0f);   // anti-windup clamp
  float I = KI * integral;

  float D = KD * (error - previousError);
  previousError = error;

  return P + I + D;
}

void resetPID() {
  integral      = 0.0f;
  previousError = 0.0f;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // ---- Left motor -> channel B ----
  if (leftSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWMB, leftSpeed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    ledcWrite(PWMB, abs(leftSpeed));
  }

  // ---- Right motor -> channel A ----
  if (rightSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWMA, rightSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWMA, abs(rightSpeed));
  }
}

void stopMotors() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMB, 0);
  ledcWrite(PWMA, 0);
}
