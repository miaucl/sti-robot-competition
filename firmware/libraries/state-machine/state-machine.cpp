/*
  state-machine.cpp - State machine functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"
#include "state-estimator.h"

/* Random Navigation Mode */
State modeRandomNavigation( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean flags[FLAG_COUNT],
                            StateEstimator *stateEstimator);

/* Platform Mode */
State modePlatform( State currentState,
                    long stateChangeTimestamp,
                    boolean btnState[BTN_COUNT],
                    boolean flags[FLAG_COUNT],
                    StateEstimator *stateEstimator);

/* Test Mode */
State modeTest( State currentState,
                long stateChangeTimestamp,
                boolean btnState[BTN_COUNT],
                boolean flags[FLAG_COUNT],
                StateEstimator *stateEstimator);


State checkStateTransition( State currentState,
                            Mode mode,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean flags[FLAG_COUNT],
                            StateEstimator *stateEstimator)
{
  switch (mode)
  {
    case m_random_navigation: return modeRandomNavigation(currentState, stateChangeTimestamp, btnState, flags, stateEstimator);
    case m_poi_navigation:    return modeRandomNavigation(currentState, stateChangeTimestamp, btnState, flags, stateEstimator);
    case m_platform:          return modePlatform(currentState, stateChangeTimestamp, btnState, flags, stateEstimator);
    case m_test:              return modeTest(currentState, stateChangeTimestamp, btnState, flags, stateEstimator);
    default:                  return modeTest(currentState, stateChangeTimestamp, btnState, flags, stateEstimator);
  }
}


State modeRandomNavigation( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean flags[FLAG_COUNT],
                            StateEstimator *stateEstimator)
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
      return s_wander;
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
      return s_wander;

      //return s_following;
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
      return s_returning;
    }
    // Exit turning state and enter following state to follow wall
    else if (flags[FLAG_SWALLOW_TIMEOUT] && stateTransitionAllowed)
    {
      flags[FLAG_SWALLOW_TIMEOUT] = 0;

      return s_scanning;
    }
  }

  /**
   * Current state "s_returning"
   */
  else if (currentState == s_returning)
  {
    // Exit swallow state and enter return state to go back
    if (flags[FLAG_RETURNED] && stateTransitionAllowed)
    {
      flags[FLAG_RETURNED] = 0;

      stateEstimator->correctPos(CORRECT_RETURNING_SWALLOWED_X,CORRECT_RETURNING_SWALLOWED_Y);

      return s_emptying;
    }
  }


  /**
   * Current state "s_returning"
   */
  else if (currentState == s_emptying)
  {
    // Exit swallow state and enter return state to go back
    if (flags[FLAG_EMPTYIED_BOTTLE] && stateTransitionAllowed)
    {
      flags[FLAG_EMPTYIED_BOTTLE] = 0;

      flags[FLAG_TURN_DOUBLE] = 1;
      return s_turning;
    }
  }

  // If no state transition, return same state
  return currentState;
}

State modePlatform( State currentState,
                    long stateChangeTimestamp,
                    boolean btnState[BTN_COUNT],
                    boolean flags[FLAG_COUNT],
                    StateEstimator *stateEstimator)
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
      return s_following;
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
   * Current state "s_following_slope"
   */
  else if (currentState == s_following_slope)
  {
    // Exit following slope state and enter …
    if (flags[FLAG_FOLLOWING_CORNER_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_FOLLOWING_CORNER_DETECTED] = 0;

      flags[FLAG_ON_PLATFORM] = 1;

      return s_test;
    }
  }

  /**
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    // Exit turning state and enter following slope state to follow wall with a ramp
    if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;

      return s_following_slope;
    }
  }

  // If no state transition, return same state
  return currentState;
}



State modeTest( State currentState,
                long stateChangeTimestamp,
                boolean btnState[BTN_COUNT],
                boolean flags[FLAG_COUNT],
                StateEstimator *stateEstimator)
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
      return s_following;
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
      flags[FLAG_TURN_DOUBLE] = 1;

      if (!flags[FLAG_SWALLOWED_BOTTLE])
      {
        stateEstimator->correctPos(CORRECT_FOLLOWING_EMPTY_X, CORRECT_FOLLOWING_EMPTY_Y);
      }
      else
      {
        stateEstimator->correctPos(CORRECT_FOLLOWING_SWALLOWED_X, CORRECT_FOLLOWING_SWALLOWED_Y);
      }


      return s_turning;
    }
  }

  /**
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    // Exit turning state and enter following slope state to follow wall with a ramp
    if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;
      flags[FLAG_SWALLOWED_BOTTLE] = !flags[FLAG_SWALLOWED_BOTTLE];

      return s_following;
    }
  }



  // If no state transition, return same state
  return currentState;
}
