/*
  state-turning.h - Turns a given angle
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_TURNING_h
#define STATE_TURNING_h

/**
 * Enter the state <state>
 */
void stateTurningEnterRoutine(boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT]);

/**
 * Run the state <state>
 */
void stateTurningRoutine(float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT],
                        boolean flags[FLAG_COUNT]);

/**
 * Exit the state <state>
 */
void stateTurningExitRoutine(boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT]);


#endif
