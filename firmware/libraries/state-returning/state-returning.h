/*
  state-returning.h - Returns to the recycling zone
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_RETURNING_h
#define STATE_RETURNING_h

/**
 * Enter the state <state>
 */
void stateReturningEnterRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);

/**
 * Run the state <state>
 */
void stateReturningRoutine( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                            int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                            int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                            float estimatedAngle,
                            double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT]);

/**
 * Exit the state <state>
 */
void stateReturningExitRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);


#endif
