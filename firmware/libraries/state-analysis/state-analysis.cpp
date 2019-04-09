/*
  state-analysis.h - Analysis state methods
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "math.h"
#include "config.h"
#include "sensors.h"
#include "actuators.h"

// Internal control flags
static bool moving = 0;


void stateAnalysisEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

}

void stateAnalysisRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                          int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                          boolean ledState[LED_COUNT])
{

}


void stateAnalysisExitRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;

}
