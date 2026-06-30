// =============================================================================
//  config.h  -  Central configuration for the Xbotix line follower
//  All pin maps, motion constants and tuning values live here so the rest of
//  the firmware never hard-codes a number.
//
//  Target : ESP32-S3 (GPIO numbers below assume an S3 dev board)
//  Core   : Arduino-ESP32 v3.x  (required for the 3-arg ledcAttach() API)
// =============================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ----- Motor encoders (wired, reserved for future odometry; not yet used) ----
#define ENC1_A 35
#define ENC1_B 36
#define ENC2_A 37
#define ENC2_B 38

// ----- TB6612FNG motor driver -----------------------------------------------
#define PWMA 6
#define AIN2 15
#define AIN1 7
#define STBY 16
#define BIN1 18
#define BIN2 17
#define PWMB 8

// ----- CD74HCxx multiplexer (8 IR sensors) ----------------------------------
#define MUX_SIG 46
#define MUX_S0  12
#define MUX_S1  11
#define MUX_S2  10
#define MUX_S3  9     // only needed for 16-ch (4067); held LOW for 8-ch use

// ----- PWM configuration -----------------------------------------------------
constexpr int PWM_FREQ = 20000;   // 20 kHz (inaudible)
constexpr int PWM_RES  = 8;       // 8-bit -> 0..255 duty

// ----- Motor speeds ----------------------------------------------------------
constexpr int BASE_SPEED          = 65;
constexpr int MODERATE_TURN_SPEED = 55;
constexpr int SHARP_TURN_SPEED    = 42;
constexpr int EXTREME_TURN_SPEED  = 30;
constexpr int MAX_SPEED           = 140;
constexpr int MIN_SPEED           = 0;
constexpr int JUNCTION_TURN_SPEED = 90;
constexpr int START_SPEED         = 60;   // initial 2 s forward push

// ----- Junction / dead-end detection ----------------------------------------
constexpr int DEAD_END_THRESHOLD    = 3;     // consecutive all-white reads
constexpr int TURN_SPEED            = 120;   // in-place recovery turn
constexpr int MIN_TURN_TIME         = 500;   // ms before a turn can "find" line
constexpr int MAX_TURN_TIME         = 3000;  // ms turn timeout
constexpr int LINE_DETECT_THRESHOLD = 3;     // sensors needed to confirm line
constexpr int JUNCTION_THRESHOLD    = 6;     // sensors => wide junction

// ----- Direction multiplier (flip if the robot steers the wrong way) --------
constexpr int DIRECTION_MULTIPLIER = -1;

// ----- PID constants ---------------------------------------------------------
constexpr float KP = 30.0f;
constexpr float KI = 0.15f;
constexpr float KD = 20.0f;

// ----- Telemetry -------------------------------------------------------------
//  Set to 0 for an official competition run. The rules require fully
//  autonomous operation with no tether, so debug serial should be OFF during
//  a scored attempt (it also speeds up the control loop).
#define ENABLE_TELEMETRY 1
#define SERIAL_BAUD      115200

#endif // CONFIG_H
