// =============================================================================
//  Telemetry.h  -  USB-serial debug output
//
//  IMPORTANT (rules compliance): this is one-way debug telemetry over the USB
//  serial port for CALIBRATION / DEVELOPMENT only. The competition requires
//  fully autonomous operation with no wired/wireless link during a scored run.
//  Set ENABLE_TELEMETRY to 0 in config.h before an official attempt; every
//  function below then compiles to a no-op (and the loop runs faster).
//
//  There is no radio/IoT link in this firmware. This "communication" module
//  exists only as the serial-debug channel.
// =============================================================================
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "config.h"

void telemetryInit();
void telemetryBanner();

// One status line per loop: state + sensor bar + black count.
void telemetryStatus(int state, const int sensors[8], int blackCount, bool allWhite);

// Boxed event message (state transitions, dead ends, junctions...).
void telemetryEvent(const char *line1, const char *line2 = nullptr);

// Per-step detail while following the line.
void telemetryFollow(float position, float error,
                     int leftSpeed, int rightSpeed,
                     int blackCount, float absError);

#endif // TELEMETRY_H
