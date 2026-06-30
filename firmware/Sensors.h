// =============================================================================
//  Sensors.h  -  8-channel IR array read through a CD74HCxx multiplexer
//
//  Scope: line-position sensing only. Colour detection, the quantum-tower
//  lamp reading and energy-cell sensing are NOT implemented yet.
// =============================================================================
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Configure mux + encoder pins. Call once from setup().
void sensorsInit();

// Read all 8 IR channels into sensors[8] (HIGH = line/black, LOW = white).
void readAllSensors(int sensors[8]);

// Compute a signed line position from a sensor frame.
//   blackCount -> number of sensors over the line
//   allWhite   -> true if no sensor sees the line (returns 999 in that case)
// Returns a non-linear, edge-weighted position centred on 0.0.
float getLinePosition(int sensors[8], int &blackCount, bool &allWhite);

#endif // SENSORS_H
