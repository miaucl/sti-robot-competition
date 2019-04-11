/*
  state-wander.h - Wander state methods
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_WANDER_h
#define STATE_WANDER_h

/**
 * Enter the state wander
 */
void stateWanderEnterRoutine(boolean ledState[LED_COUNT]);

/**
 * Run the state wander
 */
void stateWanderRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                        int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                        float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT]);

/**
 * Exit the state wander
 */
void stateWanderExitRoutine(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            boolean ledState[LED_COUNT]);


#endif
