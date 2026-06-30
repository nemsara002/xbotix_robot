// =============================================================================
//  states.h  -  Finite-state-machine states for the navigation loop
// =============================================================================
#ifndef ROBOT_STATES_H
#define ROBOT_STATES_H

enum RobotState {
  STARTUP_FORWARD,    // initial 2 s forward push out of the start square
  FOLLOWING_LINE,     // normal PID line following
  JUNCTION_FORWARD,   // nudge forward across a detected junction
  JUNCTION_TURN_LEFT, // turn left to take the junction branch
  DEAD_END_STOPPED,   // brief stop on a dead end
  TURNING_LEFT        // in-place recovery turn to re-acquire the line
};

#endif // ROBOT_STATES_H
