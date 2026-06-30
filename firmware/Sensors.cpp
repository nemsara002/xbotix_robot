// =============================================================================
//  Sensors.cpp
// =============================================================================
#include "Sensors.h"
#include "config.h"

// Drive the mux address lines (S0..S2 select channels 0..7).
static void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0,  ch       & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
}

// Select a channel and return its digital value.
static int readIRSensor(uint8_t channel) {
  selectMuxChannel(channel);
  delayMicroseconds(10);          // settle time for the mux
  return digitalRead(MUX_SIG);
}

void sensorsInit() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  // Hold the 4th address line low so a 16-ch mux stays on channels 0..7.
  // (Harmless if an 8-ch mux is fitted.)
  digitalWrite(MUX_S3, LOW);

  // Encoder inputs are wired and reserved for odometry; not read yet.
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
}

void readAllSensors(int sensors[8]) {
  for (int i = 0; i < 8; i++) {
    sensors[i] = readIRSensor(i);
  }
}

float getLinePosition(int sensors[8], int &blackCount, bool &allWhite) {
  int leftEdge  = -1;
  int rightEdge = -1;
  blackCount = 0;
  allWhite   = true;

  for (int i = 0; i < 8; i++) {
    if (sensors[i] == HIGH) {
      blackCount++;
      allWhite = false;
      if (leftEdge == -1) leftEdge = i;
      rightEdge = i;
    }
  }

  if (allWhite) return 999.0f;

  // Centre of the detected line span, offset so the array centre is 0.
  float lineCenter = (leftEdge + rightEdge) / 2.0f;
  float position   = lineCenter - 3.5f;

  // Non-linear gain: the further off-centre, the harder we react. This makes
  // sharp corners turn in tighter without over-reacting near the centre.
  float absPos = fabs(position);
  if      (absPos > 3.0f) position *= 1.9f;
  else if (absPos > 2.5f) position *= 1.6f;
  else if (absPos > 2.0f) position *= 1.4f;
  else if (absPos > 1.5f) position *= 1.2f;

  return position;
}
