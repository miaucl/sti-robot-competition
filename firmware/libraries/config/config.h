/*
  config.h - Configs and MACROs
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Config_h
#define Config_h

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

/* Sensors */
#define SENSOR_PROXIMITY_COUNT 10
#define SENSOR_PROXIMITY_CALIBRATION_COUNT 10
#define SENSOR_PROXIMITY_MEASUREMENT_COUNT 8
#define SENSOR_PROXIMITY_THRESHOLD 400
#define SENSOR_PROXIMITY_RIGHT 0
#define SENSOR_PROXIMITY_FORWARD_RIGHT 1
#define SENSOR_PROXIMITY_FORWARD 2
#define SENSOR_PROXIMITY_FORWARD_LEFT 3
#define SENSOR_PROXIMITY_LEFT 4
#define SENSOR_PROXIMITY_BACKWARD 5
#define SENSOR_PROXIMITY_DOWN_RIGHT 6
#define SENSOR_PROXIMITY_DOWN_LEFT 7
#define SENSOR_PROXIMITY_DETECT_RIGHT 8
#define SENSOR_PROXIMITY_DETECT_LEFT 9

#define SENSOR_TOF_COUNT 3
#define SENSOR_TOF_TIMEOUT 4000
#define SENSOR_TOF_CONVERTION_FACTOR 29.1
#define SENSOR_TOF_MEASUREMENT_COUNT 5
#define SENSOR_TOF_RIGHT 0
#define SENSOR_CENTER_RIGHT 1
#define SENSOR_RIGHT_RIGHT 2

#define SENSOR_IMU_COUNT 1
#define SENSOR_IMU_SDA_PIN 20
#define SENSOR_IMU_SCL_PIN 21
#define SENSOR_IMU_INT_PIN 2

#define SENSOR_IMU_MEASUREMENT_DIMENSIONS 3
#define SENSOR_IMU_MEASUREMENT_COUNT 12
#define SENSOR_IMU_YAW 0
#define SENSOR_IMU_PITCH 1
#define SENSOR_IMU_ROLL 2


#define BTN_COUNT 3
#define BTN_RESET 0
#define BTN_START 1
#define BTN_PAUSE 2

#define LED_COUNT 2
#define LED_SYSTEM 0
#define LED_RUNNING 1


/* PINs */
#define TESTLED 13
#define SENSOR_PROXIMITY_RIGHT_PIN A0
#define SENSOR_PROXIMITY_FORWARD_RIGHT_PIN A1
#define SENSOR_PROXIMITY_FORWARD_PIN A2
#define SENSOR_PROXIMITY_FORWARD_LEFT_PIN A3
#define SENSOR_PROXIMITY_LEFT_PIN A4
#define SENSOR_PROXIMITY_BACKWARD_PIN A5
#define SENSOR_PROXIMITY_DOWN_RIGHT_PIN A6
#define SENSOR_PROXIMITY_DOWN_LEFT_PIN A7
#define SENSOR_PROXIMITY_DETECT_RIGHT_PIN A8
#define SENSOR_PROXIMITY_DETECT_LEFT_PIN A9

#define SENSOR_TOF_RIGHT_TRIGGER_PIN 52
#define SENSOR_TOF_RIGHT_ECHO_PIN 53
#define SENSOR_TOF_CENTER_TRIGGER_PIN 54
#define SENSOR_TOF_CENTER_ECHO_PIN 55
#define SENSOR_TOF_LEFT_TRIGGER_PIN 56
#define SENSOR_TOF_LEFT_ECHO_PIN 57

#define GLOBAL_PAUSE_BTN 22
#define GLOBAL_PAUSE_LED 10

#define BTN_START_PIN 23

#define LED_SYSTEM_PIN 12
#define LED_RUNNING_PIN 11


/**
 * The possible states of the robot can have
 */
enum State
{
  s_initialization,       // Initialization state, only active during set up
  s_calibration,          // Calibrate all sensors and actuators
  s_idle,                 // Idle state, waiting for start
  s_test,                 // Testing state
  s_run,
  s_turn,
  s_wait,
  s_panic             // Panic state, when something unexpected happened
};


#endif
