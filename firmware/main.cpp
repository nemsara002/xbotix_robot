// =============================================================================
//  main.cpp  -  Xbotix University Category robot, navigation firmware
//
//  Implements: autonomous line following with non-linear PID steering, plus
//  junction handling, dead-end detection and an in-place recovery turn.
//
//  NOT yet implemented (no code for these tasks exists in this build):
//    - Task 1 cube pick/place        - Task 2 quantum-tower lamp reading
//    - Task 3 wall following + grip   - Task 4 matrix calc + ball throwing
//    - Task 5 energy-cell sensing/placement
//
//  Build: PlatformIO (esp32, Arduino-ESP32 v3.x). See platformio.ini.
// =============================================================================
#include <Arduino.h>
#include "config.h"
#include "states.h"
#include "Sensors.h"
#include "MotorControl.h"
#include "Telemetry.h"

// ----- Navigation bookkeeping ------------------------------------------------
static int          allWhiteCount    = 0;
static bool         wasFollowingLine = false;
static bool         junctionDetected = false;
static RobotState   currentState     = STARTUP_FORWARD;
static unsigned long stateChangeTime = 0;

static void handleNormalLineFollowing(float position, int blackCount);

// =============================================================================
void setup() {
  telemetryInit();
  motorsInit();
  sensorsInit();

  telemetryBanner();
  stateChangeTime = millis();   // start the 2 s startup-forward timer
}

// =============================================================================
void loop() {
  int sensors[8];
  readAllSensors(sensors);

  int   blackCount = 0;
  bool  allWhite   = false;
  float position   = getLinePosition(sensors, blackCount, allWhite);

  telemetryStatus(currentState, sensors, blackCount, allWhite);

  // ---- Junction detection (only while normally following) ----
  if (currentState == FOLLOWING_LINE &&
      blackCount >= JUNCTION_THRESHOLD && !junctionDetected) {
    telemetryEvent("JUNCTION DETECTED", "moving forward first");
    currentState     = JUNCTION_FORWARD;
    stateChangeTime  = millis();
    junctionDetected = true;
    allWhiteCount    = 0;
    return;                       // re-evaluate next loop in the new state
  }

  // ---- Line-tracking bookkeeping ----
  if (currentState == FOLLOWING_LINE) {
    if (blackCount >= 2 && blackCount < JUNCTION_THRESHOLD) {
      wasFollowingLine = true;
      allWhiteCount    = 0;
      junctionDetected = false;
    } else if (blackCount == 0 && wasFollowingLine) {
      allWhiteCount++;
    }
  }

  switch (currentState) {

    // -------------------------------------------------------------------------
    case STARTUP_FORWARD: {
      if (millis() - stateChangeTime < 2000) {
        setMotorSpeeds(START_SPEED, START_SPEED);
      } else {
        telemetryEvent("STARTUP COMPLETE", "starting line following");
        currentState     = FOLLOWING_LINE;
        wasFollowingLine = false;
        allWhiteCount    = 0;
      }
      break;
    }

    // -------------------------------------------------------------------------
    case FOLLOWING_LINE: {
      if (blackCount == 0 && wasFollowingLine && !junctionDetected) {
        // Lost the line after having been on it.
        if (allWhiteCount >= DEAD_END_THRESHOLD) {
          telemetryEvent("DEAD-END - stopping");
          stopMotors();
          currentState    = DEAD_END_STOPPED;
          stateChangeTime = millis();
        } else {
          setMotorSpeeds(30, 30);          // creep, hoping to re-acquire
        }
      } else if (blackCount > 0 && blackCount < JUNCTION_THRESHOLD) {
        allWhiteCount = 0;
        handleNormalLineFollowing(position, blackCount);
      } else if (blackCount == 0) {
        setMotorSpeeds(40, 40);            // searching (never saw the line yet)
      }
      break;
    }

    // -------------------------------------------------------------------------
    case JUNCTION_FORWARD: {
      if (millis() - stateChangeTime < 200) {
        setMotorSpeeds(45, 45);
      } else {
        telemetryEvent("turning left at junction");
        currentState    = JUNCTION_TURN_LEFT;
        stateChangeTime = millis();
        resetPID();
      }
      break;
    }

    // -------------------------------------------------------------------------
    case JUNCTION_TURN_LEFT: {
      unsigned long t = millis() - stateChangeTime;
      if (t < 500) {
        setMotorSpeeds(-JUNCTION_TURN_SPEED, JUNCTION_TURN_SPEED);
      } else if (blackCount >= 2 && blackCount < JUNCTION_THRESHOLD) {
        telemetryEvent("LEFT PATH FOUND", "resuming line following");
        currentState     = FOLLOWING_LINE;
        wasFollowingLine = true;
        allWhiteCount    = 0;
        junctionDetected = false;
      } else if (t > 1500) {
        telemetryEvent("junction turn timeout - resuming");
        currentState     = FOLLOWING_LINE;
        wasFollowingLine = false;
        junctionDetected = false;
      } else {
        setMotorSpeeds(-JUNCTION_TURN_SPEED, JUNCTION_TURN_SPEED);
      }
      break;
    }

    // -------------------------------------------------------------------------
    case DEAD_END_STOPPED: {
      stopMotors();
      if (millis() - stateChangeTime >= 500) {
        telemetryEvent("turning left to find line");
        currentState    = TURNING_LEFT;
        stateChangeTime = millis();
        resetPID();
      }
      break;
    }

    // -------------------------------------------------------------------------
    case TURNING_LEFT: {
      unsigned long t = millis() - stateChangeTime;
      bool lineFound  = (t > MIN_TURN_TIME && blackCount >= LINE_DETECT_THRESHOLD);

      if (lineFound || t > MAX_TURN_TIME) {
        telemetryEvent(t > MAX_TURN_TIME ? "turn timeout" : "LINE FOUND",
                       "resuming line following");
        currentState     = FOLLOWING_LINE;
        wasFollowingLine = true;
        allWhiteCount    = 0;
      } else {
        setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
      }
      break;
    }
  }

  delay(10);   // ~100 Hz control loop
}

// =============================================================================
//  Normal PID line following with speed scaling by turn severity.
// =============================================================================
static void handleNormalLineFollowing(float position, int blackCount) {
  float error    = -position * DIRECTION_MULTIPLIER;
  float absError  = fabs(error);
  float control   = calculateControl(error);

  bool isExtremeTurn   = (absError > 3.3f) && (blackCount == 1);
  bool isVerySharpTurn = (absError > 2.8f) && (blackCount <= 2);
  bool isSharpTurn     = (absError > 2.3f);
  bool isModerateTurn  = (absError > 1.5f);

  int baseSpeed;
  if      (isExtremeTurn)   baseSpeed = EXTREME_TURN_SPEED;
  else if (isVerySharpTurn) baseSpeed = SHARP_TURN_SPEED;
  else if (isSharpTurn)     baseSpeed = MODERATE_TURN_SPEED;
  else if (isModerateTurn)  baseSpeed = 58;
  else                      baseSpeed = BASE_SPEED;

  int leftSpeed  = baseSpeed - control;
  int rightSpeed = baseSpeed + control;

  // Per-severity clamps: tighter turns may reverse a wheel, gentle ones can't.
  if (isExtremeTurn) {
    leftSpeed  = constrain(leftSpeed,  -90, 120);
    rightSpeed = constrain(rightSpeed, -90, 120);
  } else if (isVerySharpTurn) {
    leftSpeed  = constrain(leftSpeed,  -60, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -60, MAX_SPEED);
  } else if (isSharpTurn) {
    leftSpeed  = constrain(leftSpeed,  -30, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -30, MAX_SPEED);
  } else if (isModerateTurn) {
    leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
  } else {
    leftSpeed  = constrain(leftSpeed,  baseSpeed - 28, baseSpeed + 28);
    rightSpeed = constrain(rightSpeed, baseSpeed - 28, baseSpeed + 28);
  }

  setMotorSpeeds(leftSpeed, rightSpeed);
  telemetryFollow(position, error, leftSpeed, rightSpeed, blackCount, absError);
}
