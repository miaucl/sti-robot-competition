/*
  state-machine.cpp - State machine functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"


State checkStateTransition( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean flags[FLAG_COUNT])
{
  boolean stateTransitionAllowed;
  #ifdef MANUAL_STATE_TRANSITION_ENABLE
  stateTransitionAllowed = btnState[BTN_STATE];
  #else
  stateTransitionAllowed = true;
  #endif


  /**
   * Current state "s_initialization"
   */
  if (currentState == s_initialization)
  {
    // Perform an automatic transition to the idle state once the system is initialized
    return s_idle;
  }


  /**
   * Current state "s_idle"
   */
  else if (currentState == s_idle)
  {
    // Wait for the START button to be pressed to calibrate the robot
    if (btnState[BTN_START])
    {
      return s_calibration;
    }
  }


  /**
   * Current state "s_calibration"
   */
  else if (currentState == s_calibration)
  {
    // Wait for the START button to be pressed to start the robot
    if (btnState[BTN_START])
    {
      return s_swallowing;
    }
  }

  /**
   * Current state "s_wander"
   */
  else if (currentState == s_wander)
  {
    // Exit wander state and enter scanning state to check the element
    if (flags[FLAG_ELEMENT_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_ELEMENT_DETECTED] = 0;

      return s_scanning;
    }
  }

  /**
   * Current state "s_scanning"
   */
  else if (currentState == s_scanning)
  {
    // Exit scanning state and enter wander state when nothing has been scanned
    if (flags[FLAG_NOTHING_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_NOTHING_DETECTED] = 0;

      return s_wander;
    }
    // Exit scanning state and enter wander state when a wall has been scanned
    else if (flags[FLAG_WALL_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_WALL_DETECTED] = 0;

      return s_wander;
    }
    // Exit scanning state and enter swallowing state when a bottle has been scanned
    else if (flags[FLAG_BOTTLE_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_BOTTLE_DETECTED] = 0;

      return s_swallowing;
    }
  }


  /**
   * Current state "s_following"
   */
  else if (currentState == s_following)
  {
    // Exit following state and enter turn state to follow corner
    if (flags[FLAG_FOLLOWING_CORNER_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_FOLLOWING_CORNER_DETECTED] = 0;

      flags[FLAG_TURN_RIGHT] = !flags[FLAG_FOLLOWING_RIGHT_SIDE];

      return s_turning;
    }
  }

  /**
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    // Exit turning state and enter following state to follow wall
    if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;

      return s_following;
    }
  }

  /**
   * Current state "s_swallowing"
   */
  else if (currentState == s_swallowing)
  {
    // Exit swallow state and enter return state to go back
    if (flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed)
    {
      return;
    }
    // Exit turning state and enter following state to follow wall
    else if (flags[FLAG_SWALLOW_TIMEOUT] && stateTransitionAllowed)
    {
      flags[FLAG_SWALLOW_TIMEOUT] = 0;

      return s_scanning;
    }
  }





  // If no state transition, return same state
  return currentState;
}
