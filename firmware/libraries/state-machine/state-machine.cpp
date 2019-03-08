/*
  state-machine.cpp - State machine functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"


State checkStateTransition( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT])
{
  /**
   * Current state "s_initialization"
   */
  if (currentState == s_initialization)
  {
    // Perform an automatic transition to the calibration state once the system is initialized
    return s_calibration;
  }





  /**
   * Current state "s_calibration"
   */
  else if (currentState == s_calibration)
  {
    // Perform an automatic transition to the idle state once the system is calibrated
    return s_idle;
  }





  /**
   * Current state "s_idle"
   */
  else if (currentState == s_idle)
  {
    // Wait for the START button to be pressed to start the robot
    if (btnState[BTN_START])
    {
      return s_test; // <================================ TESTING
    }
  }

  // If no state transition, return same state
  return currentState;
}