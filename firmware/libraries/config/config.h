/*
  config.h - Configs and MACROs
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Config_h
#define Config_h

#include "Arduino.h"

/**
 * MACROS
 */

/* The unique ID of the robot */
#define ROBOT_ID 1

/* Serial communication */
#define SERIAL_ENABLE 1
#define SERIAL_BAUD_RATE 19200

/* Timings */
#define SENSOR_PROXIMITY_FREQUENCY 10 // ms
#define SENSORS_FREQUENCY 100 // ms
#define CONTROLLER_FREQUENCY 100 // ms

/* Thresholds */
#define THRESHOLD_HORIZONTAL_PROXIMITY 60

/* Sensors */
#define SENSOR_PROXIMITY_RIGHT 1
#define SENSOR_PROXIMITY_FORWARD_RIGHT 2
#define SENSOR_PROXIMITY_FORWARD 3
#define SENSOR_PROXIMITY_FORWARD_LEFT 4
#define SENSOR_PROXIMITY_LEFT 5
#define SENSOR_PROXIMITY_BACKWARD 6

/* PINs */
#define TESTLED 13
#define SENSOR_PROXIMITY_RIGHT_PIN A0
#define SENSOR_PROXIMITY_FORWARD_RIGHT_PIN A1
#define SENSOR_PROXIMITY_FORWARD_PIN A2
#define SENSOR_PROXIMITY_FORWARD_LEFT_PIN A3
#define SENSOR_PROXIMITY_LEFT_PIN A4
#define SENSOR_PROXIMITY_BACKWARD_PIN A5

/**
 * The possible states of the robot can have
 */
enum State
{
  s_initialize,       // Initialization state, only active during set up
  s_run,
  s_turn,
  s_wait,
  s_panic             // Panic state, when something unexpected happened
};


#endif
