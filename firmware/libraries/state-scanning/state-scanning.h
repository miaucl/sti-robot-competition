/*
  state-scanning.h - Scanning state methods
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_SCANNING_h
#define STATE_SCANNING_h

/**
 * Enter the state scanning
 */
void stateScanningEnterRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);

/**
 * Run the state scanning
 */
void stateScanningRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
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
 * Exit the state scanning
 */
void stateScanningExitRoutine(boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT]);


#endif
