/*
  state-machine.cpp - State machine functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"
#include "state-estimator.h"
#include "leds.h"

/* Random Navigation Mode */
State modeRandomNavigation( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT],
                            StateEstimator *stateEstimator);

/* POI Navigation Mode */
State modePOINavigation(State currentState,
                        long stateChangeTimestamp,
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT],
                        boolean flags[FLAG_COUNT],
                        StateEstimator *stateEstimator);

/* Platform Mode */
State modePlatform( State currentState,
                    long stateChangeTimestamp,
                    boolean btnState[BTN_COUNT],
                    boolean ledState[LED_COUNT],
                    boolean flags[FLAG_COUNT],
                    StateEstimator *stateEstimator);

/* Collect Mode */
State modeCollect(State currentState,
                  long stateChangeTimestamp,
                  boolean btnState[BTN_COUNT],
                  boolean ledState[LED_COUNT],
                  boolean flags[FLAG_COUNT],
                  StateEstimator *stateEstimator);

/* Test Mode */
State modeTest( State currentState,
                long stateChangeTimestamp,
                boolean btnState[BTN_COUNT],
                boolean ledState[LED_COUNT],
                boolean flags[FLAG_COUNT],
                StateEstimator *stateEstimator);


State checkStateTransition( State currentState,
                            Mode mode,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT],
                            StateEstimator *stateEstimator)
{
  switch (mode)
  {
    case m_random_navigation: return modeRandomNavigation(currentState, stateChangeTimestamp, btnState, ledState, flags, stateEstimator);
    case m_poi_navigation:    return modePOINavigation(currentState, stateChangeTimestamp, btnState, ledState, flags, stateEstimator);
    case m_platform:          return modePlatform(currentState, stateChangeTimestamp, btnState, ledState, flags, stateEstimator);
    case m_collect:           return modeCollect(currentState, stateChangeTimestamp, btnState, ledState, flags, stateEstimator);
    case m_test:              return modeTest(currentState, stateChangeTimestamp, btnState, ledState, flags, stateEstimator);
    default:                  return modeTest(currentState, stateChangeTimestamp, btnState, ledState, flags, stateEstimator);
  }
}


// State BUTTON path: -> wander -> returning
State modeRandomNavigation( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT],
                            StateEstimator *stateEstimator)
{
  boolean stateTransitionAllowed;
  boolean nextState = false;
  static boolean nextStateDone = false;

  #ifdef MANUAL_STATE_TRANSITION_ENABLE
  stateTransitionAllowed = btnState[BTN_STATE];
  #else
  stateTransitionAllowed = true;

  if (btnState[BTN_STATE])
  {
    if (!nextStateDone)
    {
      nextState = true;
      nextStateDone = true;
    }
    else
    {
      return currentState;
    }
  }
  else
  {
    if (nextStateDone)
    {
      ledState[LED_NEXT_STATE] = 1;
      writeLeds(ledState);
      delay(500);
      ledState[LED_NEXT_STATE] = 0;
      writeLeds(ledState);

      nextStateDone = false;
    }
  }
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
    if ((btnState[BTN_START]) || nextState)
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
    if ((flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed) || nextState)
    {
      flags[FLAG_SWALLOWED_BOTTLE] = 1;
      return s_returning;
    }
    else if (flags[FLAG_ELEMENT_DETECTED] && stateTransitionAllowed)
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
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    // Exit turning state and enter following state to follow wall
    if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;
      return s_wander;
    }
  }

  /**
   * Current state "s_swallowing"
   */
  else if (currentState == s_swallowing)
  {
    // Exit swallow state and turn
    if (flags[FLAG_STONES_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_STONES_DETECTED] = 0;
      flags[FLAG_TURN_DOUBLE] = 1;

      return s_turning;
    }
    // Exit swallow state and wander
    else if (flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed)
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
   * Current state "s_emptying"
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

// State BUTTON path: -> poi -> wander -> returning
State modePOINavigation(State currentState,
                        long stateChangeTimestamp,
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT],
                        boolean flags[FLAG_COUNT],
                        StateEstimator *stateEstimator)
{
  boolean stateTransitionAllowed;
  boolean nextState = false;
  static boolean nextStateDone = false;

  #ifdef MANUAL_STATE_TRANSITION_ENABLE
  stateTransitionAllowed = btnState[BTN_STATE];
  #else
  stateTransitionAllowed = true;

  if (btnState[BTN_STATE])
  {
    if (!nextStateDone)
    {
      nextState = true;
      nextStateDone = true;
    }
    else
    {
      return currentState;
    }
  }
  else
  {
    if (nextStateDone)
    {
      ledState[LED_NEXT_STATE] = 1;
      writeLeds(ledState);
      delay(500);
      ledState[LED_NEXT_STATE] = 0;
      writeLeds(ledState);

      nextStateDone = false;
    }
  }
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
    if ((btnState[BTN_START]) || nextState)
    {
      return s_poi;
    }
  }

  /**
   * Current state "s_poi"
   */
  else if (currentState == s_poi)
  {
    // Exit wander state and enter scanning state to check the element
    if ((flags[FLAG_POI_REACHED] && stateTransitionAllowed) || nextState)
    {
      flags[FLAG_POI_REACHED] = 0;

      return s_wander;
    }
  }


  /**
   * Current state "s_wander"
   */
  else if (currentState == s_wander)
  {
    // Exit wander state and enter scanning state to check the element
    if ((flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed) || nextState)
    {
      flags[FLAG_SWALLOWED_BOTTLE] = 1;
      return s_returning;
    }
    else if (flags[FLAG_ELEMENT_DETECTED] && stateTransitionAllowed)
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
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    // Exit turning state and enter following state to follow wall
    if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;
      return s_poi;
    }
  }

  /**
   * Current state "s_swallowing"
   */
  else if (currentState == s_swallowing)
  {
    // Exit swallow state and turn
    if (flags[FLAG_STONES_DETECTED] && stateTransitionAllowed)
    {
      flags[FLAG_STONES_DETECTED] = 0;
      flags[FLAG_TURN_DOUBLE] = 1;

      return s_turning;
    }
    // Exit swallow state and wander
    else if (flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed)
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
   * Current state "s_emptying"
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

// State BUTTON path: -> following -> following slope -> wander -> returning
State modePlatform( State currentState,
                    long stateChangeTimestamp,
                    boolean btnState[BTN_COUNT],
                    boolean ledState[LED_COUNT],
                    boolean flags[FLAG_COUNT],
                    StateEstimator *stateEstimator)
{
  boolean stateTransitionAllowed;
  boolean nextState = false;
  static boolean nextStateDone = false;

  #ifdef MANUAL_STATE_TRANSITION_ENABLE
  stateTransitionAllowed = btnState[BTN_STATE];
  #else
  stateTransitionAllowed = true;

  if (btnState[BTN_STATE])
  {
    if (!nextStateDone)
    {
      nextState = true;
      nextStateDone = true;
    }
    else
    {
      return currentState;
    }
  }
  else
  {
    if (nextStateDone)
    {
      ledState[LED_NEXT_STATE] = 1;
      writeLeds(ledState);
      delay(500);
      ledState[LED_NEXT_STATE] = 0;
      writeLeds(ledState);

      nextStateDone = false;
    }
  }
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
    if ((btnState[BTN_START]) || nextState)
    {
      //flags[FLAG_ON_PLATFORM] = 1;
      //flags[FLAG_SWALLOWED_BOTTLE] = 1;
      return s_following;
    }
  }

  /**
   * Current state "s_following"
   */
  else if (currentState == s_following)
  {
    // Exit following state and enter turn state to follow corner
    if ((flags[FLAG_FOLLOWING_CORNER_DETECTED] && stateTransitionAllowed) || nextState)
    {
      stateEstimator->correctPos(CORRECT_FOLLOWING_X,CORRECT_FOLLOWING_Y);

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
    if ((flags[FLAG_FOLLOWING_CORNER_DETECTED] && stateTransitionAllowed) || nextState)
    {
      stateEstimator->correctPos(CORRECT_FOLLOWING_SLOPE_X,CORRECT_FOLLOWING_SLOPE_Y);

      flags[FLAG_FOLLOWING_CORNER_DETECTED] = 0;

      flags[FLAG_ON_PLATFORM] = 1;

      return s_wander;
    }
  }

  /**
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    if (flags[FLAG_TURN_FINISHED] && flags[FLAG_ON_PLATFORM] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;

      return s_wander;
    }
    else if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;

      return s_following_slope;
    }
  }

  /**
   * Current state "s_wander"
   */
  else if (currentState == s_wander)
  {
    // Exit wander state and enter scanning state to check the element
    if ((flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed) || nextState)
    {
      flags[FLAG_SWALLOWED_BOTTLE] = 1;
      return s_returning;
    }
    else if (flags[FLAG_ELEMENT_DETECTED] && stateTransitionAllowed)
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
   * Current state "s_swallowing"
   */
  else if (currentState == s_swallowing)
  {
    // Exit swallow state and enter return on platform state to go back
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
    if (flags[FLAG_RETURNED] && stateTransitionAllowed)
    {
      flags[FLAG_RETURNED] = 0;

      stateEstimator->correctPos(CORRECT_RETURNING_SLOPE_X,CORRECT_RETURNING_SLOPE_Y);

      return s_slope_down;
    }
  }

  /**
   * Current state "s_slope_down"
   */
  else if (currentState == s_slope_down)
  {
    if (!flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_RIGHT] = 1;
      return s_turning;
    }
  }


  // If no state transition, return same state
  return currentState;
}


// State BUTTON path: -> following -> following collect -> returning
State modeCollect(State currentState,
                  long stateChangeTimestamp,
                  boolean btnState[BTN_COUNT],
                  boolean ledState[LED_COUNT],
                  boolean flags[FLAG_COUNT],
                  StateEstimator *stateEstimator)
{
  boolean stateTransitionAllowed;
  boolean nextState = false;
  static boolean nextStateDone = false;

  #ifdef MANUAL_STATE_TRANSITION_ENABLE
  stateTransitionAllowed = btnState[BTN_STATE];
  #else
  stateTransitionAllowed = true;

  if (btnState[BTN_STATE])
  {
    if (!nextStateDone)
    {
      nextState = true;
      nextStateDone = true;
    }
    else
    {
      return currentState;
    }
  }
  else
  {
    if (nextStateDone)
    {
      ledState[LED_NEXT_STATE] = 1;
      writeLeds(ledState);
      delay(500);
      ledState[LED_NEXT_STATE] = 0;
      writeLeds(ledState);

      nextStateDone = false;
    }
  }
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
    if ((btnState[BTN_START]) || nextState)
    {
      // flags[FLAG_ON_PLATFORM] = 1;
      // flags[FLAG_SWALLOWED_BOTTLE] = 1;
      return s_following;
    }
  }

  /**
   * Current state "s_following"
   */
  else if (currentState == s_following)
  {
    // Exit following state and enter turn state to follow corner
    if ((flags[FLAG_FOLLOWING_CORNER_DETECTED] && stateTransitionAllowed) || nextState)
    {
      stateEstimator->correctPos(CORRECT_FOLLOWING_X,CORRECT_FOLLOWING_Y);

      flags[FLAG_FOLLOWING_CORNER_DETECTED] = 0;

      flags[FLAG_TURN_RIGHT] = !flags[FLAG_FOLLOWING_RIGHT_SIDE];

      return s_turning;
    }
  }

  /**
   * Current state "s_following_collect"
   */
  else if (currentState == s_following_collect)
  {
    // Exit following slope state and enter …
    if ((flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed) || nextState)
    {
      flags[FLAG_SWALLOWED_BOTTLE] = 1;
      return s_returning;
    }
  }

  /**
   * Current state "s_turning"
   */
  else if (currentState == s_turning)
  {
    if (flags[FLAG_TURN_FINISHED] && flags[FLAG_RESET_ROBOT] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;
      flags[FLAG_RESET_ROBOT] = 0;

      return s_following;
    }
    else if (flags[FLAG_TURN_FINISHED] && stateTransitionAllowed)
    {
      flags[FLAG_TURN_FINISHED] = 0;

      return s_following_collect;
    }
  }

  /**
   * Current state "s_returning"
   */
  else if (currentState == s_returning)
  {
    // Exit swallow state and enter return state to go back
    if (flags[FLAG_RETURNED] && flags[FLAG_SWALLOWED_BOTTLE] && stateTransitionAllowed)
    {
      flags[FLAG_RETURNED] = 0;

      stateEstimator->correctPos(CORRECT_RETURNING_SWALLOWED_X,CORRECT_RETURNING_SWALLOWED_Y);

      return s_emptying;
    }
    else if (flags[FLAG_RETURNED] && stateTransitionAllowed)
    {
      flags[FLAG_RETURNED] = 0;
      flags[FLAG_RESET_ROBOT] = 1;

      return s_turning;
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

      return s_returning;
    }
  }

  // If no state transition, return same state
  return currentState;
}



State modeTest( State currentState,
                long stateChangeTimestamp,
                boolean btnState[BTN_COUNT],
                boolean ledState[LED_COUNT],
                boolean flags[FLAG_COUNT],
                StateEstimator *stateEstimator)
{
  boolean stateTransitionAllowed;
  boolean nextState = false;
  static boolean nextStateDone = false;

  #ifdef MANUAL_STATE_TRANSITION_ENABLE
  stateTransitionAllowed = btnState[BTN_STATE];
  #else
  stateTransitionAllowed = true;

  if (btnState[BTN_STATE])
  {
    if (!nextStateDone)
    {
      nextState = true;
      nextStateDone = true;
    }
    else
    {
      return currentState;
    }
  }
  else
  {
    if (nextStateDone)
    {
      ledState[LED_NEXT_STATE] = 1;
      writeLeds(ledState);
      delay(500);
      ledState[LED_NEXT_STATE] = 0;
      writeLeds(ledState);

      nextStateDone = false;
    }
  }
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
    if ((btnState[BTN_START]) || nextState)
    {
      return s_test;
    }
  }



  // If no state transition, return same state
  return currentState;
}
