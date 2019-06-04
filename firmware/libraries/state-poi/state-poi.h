/*
  state-poi.h - Go to a point of interest
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_POI_h
#define STATE_POI_h

#include "BasicLinearAlgebra.h"
using namespace BLA;


/**
 * Enter the state <state>
 */
void statePOIEnterRoutine(boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT]);

/**
 * Run the state <state>
 */
void statePOIRoutine( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                      int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                      int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                      int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                      float estimatedAngle,
                      Matrix<2> estimatedPos,
                      double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                      double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                      boolean btnState[BTN_COUNT],
                      boolean ledState[LED_COUNT],
                      boolean flags[FLAG_COUNT]);

/**
 * Exit the state <state>
 */
void statePOIExitRoutine( boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT]);


#endif
