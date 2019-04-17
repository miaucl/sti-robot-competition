/*
  state-analysis.h - Analysis state methods
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_ANALYSIS_h
#define STATE_ANALYSIS_h

/**
 * Enter the state analysis
 */
void stateAnalysisEnterRoutine(boolean ledState[LED_COUNT]);

/**
 * Run the state analysis
 */
void stateAnalysisRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                        int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                        float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT]);

/**
 * Exit the state analysis
 */
void stateAnalysisExitRoutine(boolean ledState[LED_COUNT]);


#endif
