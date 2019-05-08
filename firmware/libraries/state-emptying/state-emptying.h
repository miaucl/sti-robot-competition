/*
  state-emptying.h - Emptying the robot and leave the bottle at the recycling station
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_EMPTYING_h
#define STATE_EMPTYING_h

/**
 * Enter the state emptying
 */
void stateEmptyingEnterRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);

/**
 * Run the state emptying
 */
void stateEmptyingRoutine(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                          int servoAngles[ACTUATOR_SERVO_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT]);

/**
 * Exit the state emptying
 */
void stateEmptyingExitRoutine(boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT]);


#endif
