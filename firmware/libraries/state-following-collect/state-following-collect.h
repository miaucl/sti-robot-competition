/*
  state-following-collect.h - Following collect state methods
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_FOLLOWING_COLLECT_h
#define STATE_FOLLOWING_COLLECT_h

/**
 * Enter the state following
 */
void stateFollowingCollectEnterRoutine(boolean ledState[LED_COUNT],
                                       boolean flags[FLAG_COUNT]);

/**
 * Run the state following
 */
void stateFollowingCollectRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                                  int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                                  int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                                  int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                                  float estimatedAngle,
                                  float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                                  double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                                  double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                                  int servoAngles[ACTUATOR_SERVO_COUNT],
                                  boolean btnState[BTN_COUNT],
                                  boolean ledState[LED_COUNT],
                                  boolean flags[FLAG_COUNT]);

/**
 * Exit the state following
 */
void stateFollowingCollectExitRoutine(boolean ledState[LED_COUNT],
                                      boolean flags[FLAG_COUNT]);


#endif
