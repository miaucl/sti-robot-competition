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
#define SERIAL_BAUD_RATE 9600

/* Timings */
#define SENSOR_PROXIMITY_FREQUENCY 100 // ms
#define SENSORS_FREQUENCY 100 // ms
#define CONTROLLER_FREQUENCY 100 // ms

/* Sensors */
#define SENSOR_PROXIMITY_TEST 1

/* PINs */
#define TESTLED 13
#define SENSOR_PROXIMITY_TEST_PIN A0

#endif
