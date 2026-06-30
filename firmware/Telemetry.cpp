// =============================================================================
//  Telemetry.cpp
//  All bodies are wrapped in #if ENABLE_TELEMETRY so a competition build
//  (ENABLE_TELEMETRY 0) drops the serial traffic entirely.
// =============================================================================
#include "Telemetry.h"

void telemetryInit() {
#if ENABLE_TELEMETRY
  Serial.begin(SERIAL_BAUD);
#endif
}

void telemetryBanner() {
#if ENABLE_TELEMETRY
  Serial.println();
  Serial.println(F("=== XBOTIX LINE FOLLOWER ==="));
  Serial.println(F("Startup: 2 s forward, then line-following"));
  Serial.print(F("PID  Kp:")); Serial.print(KP);
  Serial.print(F(" Ki:"));     Serial.print(KI);
  Serial.print(F(" Kd:"));     Serial.println(KD);
  Serial.print(F("Base:"));    Serial.print(BASE_SPEED);
  Serial.print(F(" JuncTurn:")); Serial.println(JUNCTION_TURN_SPEED);
  Serial.println();
#endif
}

void telemetryStatus(int state, const int sensors[8], int blackCount, bool allWhite) {
#if ENABLE_TELEMETRY
  Serial.print(F("State:")); Serial.print(state);
  Serial.print(F(" | ["));
  for (int i = 7; i >= 0; i--) Serial.print(sensors[i] == HIGH ? '#' : '.');
  Serial.print(F("] Black:")); Serial.print(blackCount); Serial.print(F("/8"));
  if (allWhite) Serial.print(F(" | ALL WHITE"));
  Serial.println();
#endif
}

void telemetryEvent(const char *line1, const char *line2) {
#if ENABLE_TELEMETRY
  Serial.println(F("------------------------------"));
  Serial.print(F("  ")); Serial.println(line1);
  if (line2) { Serial.print(F("  ")); Serial.println(line2); }
  Serial.println(F("------------------------------"));
#endif
}

void telemetryFollow(float position, float error,
                     int leftSpeed, int rightSpeed,
                     int blackCount, float absError) {
#if ENABLE_TELEMETRY
  const char *type;
  if      ((absError > 3.3f) && (blackCount == 1)) type = "EXTREME ";
  else if ((absError > 2.8f) && (blackCount <= 2)) type = "V.SHARP ";
  else if ( absError > 2.3f)                       type = "SHARP   ";
  else if ( absError > 1.5f)                       type = "MODERATE";
  else                                             type = "STRAIGHT";

  Serial.print(F("FOLLOW ")); Serial.print(type);
  Serial.print(F(" Pos:")); Serial.print(position, 1);
  Serial.print(F(" Err:")); Serial.print(error, 1);
  Serial.print(F(" L:"));   Serial.print(leftSpeed);
  Serial.print(F(" R:"));   Serial.println(rightSpeed);
#endif
}
