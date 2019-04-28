/*
  state-following.h - Following state methods
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_ANALYSIS_h
#define STATE_ANALYSIS_h

/**
 * Enter the state following
 */
void stateFollowingEnterRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);

/**
 * Run the state following
 */
void stateFollowingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                          int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                          int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                          int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT]);

/**
 * Exit the state following
 */
void stateFollowingExitRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);


#endif
