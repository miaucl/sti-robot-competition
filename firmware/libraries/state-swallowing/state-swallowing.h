/*
  state-swallowing.h - Following state methods
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_SWALLOWING_h
#define STATE_SWALLOWING_h

/**
 * Enter the state swallowing
 */
void stateSwallowingEnterRoutine( boolean ledState[LED_COUNT],
                                  boolean flags[FLAG_COUNT]);

/**
 * Run the state swallowing
 */
void stateSwallowingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                            int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                            int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                            double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                            int servoAngles[ACTUATOR_SERVO_COUNT],
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT]);

/**
 * Exit the state swallowing
 */
void stateSwallowingExitRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT]);


#endif
