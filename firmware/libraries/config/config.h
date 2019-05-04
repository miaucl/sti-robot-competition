/*
  config.h - Configs and MACROs
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Config_h
#define Config_h

// Constants
//#define M_PI 3.141592653589793238462643


/**
 * MACROS
 */

/* The unique ID of the robot */
#define ROBOT_ID 1

/* Debugging */
#define DEBUG_ENABLE 1
//#define MANUAL_STATE_TRANSITION_ENABLE 1

/* Serial communication */
#define SERIAL_ENABLE 1
#define SERIAL_CONN_ENABLE 1
#define SERIAL_BAUD_RATE 19200

/* Timings */
#define PERIOD 10.f // ms


/* Sensors */
#define SENSOR_PROXIMITY_COUNT 10
#define SENSOR_PROXIMITY_CALIBRATION_COUNT 10
#define SENSOR_PROXIMITY_MEASUREMENT_COUNT 8
#define SENSOR_PROXIMITY_THRESHOLD 400
#define SENSOR_PROXIMITY_RIGHT 0
#define SENSOR_PROXIMITY_RIGHT_OFFSET 90
#define SENSOR_PROXIMITY_FORWARD_RIGHT 1
#define SENSOR_PROXIMITY_FORWARD_RIGHT_OFFSET 45
#define SENSOR_PROXIMITY_FORWARD 2
#define SENSOR_PROXIMITY_FORWARD_OFFSET 0
#define SENSOR_PROXIMITY_FORWARD_LEFT 3
#define SENSOR_PROXIMITY_FORWARD_LEFT_OFFSET -45
#define SENSOR_PROXIMITY_LEFT 4
#define SENSOR_PROXIMITY_LEFT_OFFSET -90
#define SENSOR_PROXIMITY_BACKWARD 5
#define SENSOR_PROXIMITY_BACKWARD_OFFSET 180
#define SENSOR_PROXIMITY_DOWN_RIGHT 6
#define SENSOR_PROXIMITY_DOWN_LEFT 7
#define SENSOR_PROXIMITY_DETECT_RIGHT 8
#define SENSOR_PROXIMITY_DETECT_LEFT 9

#define SENSOR_TOF_COUNT 3
#define SENSOR_TOF_TIMEOUT 4000 // us
#define SENSOR_TOF_CONVERTION_FACTOR 29.1
#define SENSOR_TOF_MEASUREMENT_COUNT 5
#define SENSOR_TOF_RIGHT 0
#define SENSOR_TOF_CENTER 1
#define SENSOR_TOF_LEFT 2

#define SENSOR_IMU_COUNT 1

#define SENSOR_IMU_MEASUREMENT_DIMENSIONS 3
#define SENSOR_IMU_MEASUREMENT_COUNT 12
#define SENSOR_IMU_Z_ORIENTATION_THRESHOLD 10.f
#define SENSOR_IMU_YAW 0
#define SENSOR_IMU_PITCH 1
#define SENSOR_IMU_ROLL 2
#define SENSOR_IMU_YAW_DRIFT -0.0000041f // Earth Rotational Speed


#define BTN_COUNT 4
#define BTN_RESET 0
#define BTN_START 1
#define BTN_PAUSE 2
#define BTN_STATE 3

#define LED_COUNT 8
#define LED_SYSTEM 0
#define LED_RUNNING 1
#define LED_2 2
#define LED_3 3
#define LED_4 4
#define LED_5 5
#define LED_6 6
#define LED_7 7

#define ACTUATOR_MOTOR_COUNT 2
#define ACTUATOR_MOTOR_RIGHT 0
#define ACTUATOR_MOTOR_LEFT 1
// [0,200] -> [A, B] (m/s)
#define ACTUATOR_MOTOR_SPEED_MAX 2.f // m/s
#define ACTUATOR_MOTOR_SPEED_BOUNDS_A -255.f
#define ACTUATOR_MOTOR_SPEED_BOUNDS_B 255.f
#define ACTUATOR_MOTOR_ENCODER_RESOLUTION 24
#define ACTUATOR_MOTOR_TRANSMISSION 75
#define ACTUATOR_MOTOR_DIAMETER 0.12f // m
#define ACTUATOR_MOTOR_MAX_SLEW_RATE 3.f
#define ACTUATOR_MOTOR_PID_KP 100.f
#define ACTUATOR_MOTOR_PID_KI 200.f
#define ACTUATOR_MOTOR_PID_KD 20.f

#define ACTUATOR_SERVO_COUNT 2
#define ACTUATOR_SERVO_BAR_RIGHT 0
#define ACTUATOR_SERVO_BAR_LEFT 1
#define ACTUATOR_SERVO_STEPS_PER_SECOND 60.f // steps/s

#define ACTUATOR_SERVO_BAR_RIGHT_OPEN 70
#define ACTUATOR_SERVO_BAR_RIGHT_CLOSED 10
#define ACTUATOR_SERVO_BAR_LEFT_OPEN 110
#define ACTUATOR_SERVO_BAR_LEFT_CLOSED 170

#define FLAG_COUNT 9
#define FLAG_ENABLE_ESTIMATOR 0
#define FLAG_ELEMENT_DETECTED 1
#define FLAG_BOTTLE_DETECTED 2
#define FLAG_WALL_DETECTED 3
#define FLAG_NOTHING_DETECTED 4
#define FLAG_CORNER_DETECTED 5
#define FLAG_FOLLOWING_RIGHT_SIDE 6
#define FLAG_FOLLOWING_CORNER_DETECTED 7
#define FLAG_TURN_RIGHT 8


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
#define SENSOR_TOF_CENTER_TRIGGER_PIN 50
#define SENSOR_TOF_CENTER_ECHO_PIN 51
#define SENSOR_TOF_LEFT_TRIGGER_PIN 48
#define SENSOR_TOF_LEFT_ECHO_PIN 49

#define SENSOR_IMU_SDA_PIN 20
#define SENSOR_IMU_SCL_PIN 21
#define SENSOR_IMU_INT_PIN 2

#define GLOBAL_PAUSE_BTN 22
#define GLOBAL_PAUSE_LED 10

#define BTN_START_PIN 25
#define BTN_PAUSE_PIN 22
#define BTN_STATE_PIN 23

#define LED_SYSTEM_PIN 30
#define LED_RUNNING_PIN 31
#define LED_2_PIN 32
#define LED_3_PIN 33
#define LED_4_PIN 34
#define LED_5_PIN 35
#define LED_6_PIN 36
#define LED_7_PIN 37

#define ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN 4
#define ACTUATOR_MOTOR_RIGHT_SPEED_PIN 5
#define ACTUATOR_MOTOR_RIGHT_INT_PIN 3
#define ACTUATOR_MOTOR_LEFT_DIRECTION_PIN 7
#define ACTUATOR_MOTOR_LEFT_SPEED_PIN 6
#define ACTUATOR_MOTOR_LEFT_INT_PIN 19

#define ACTUATOR_SERVO_BAR_RIGHT_PIN 11
#define ACTUATOR_SERVO_BAR_LEFT_PIN 12


/**
 * Estimator
 */
#define ESTIMATOR_CALIBRATION_MAX 200
//#define ESTIMATOR_CALIBRATION_MAX 0 // Uncomment to bypass estimator calibration
#define ESTIMATOR_CALIBRATION_COUNTER 8


/**
 * The possible states of the robot can have
 */
enum State
{
  s_initialization,       //0 Initialization state, only active during set up
  s_calibration,          //1 Calibrate all sensors and actuators
  s_idle,                 //2 Idle state, waiting for start
  s_test,                 //3 Testing state to test all features manually
  s_wander,               //4 Wander around, evade obstacles with TOF and detect bottles with IR
  s_scanning,             //5 Look for bottles (obstacles) in close range
  s_following,            //6 Use to make some following on the behaviour
  s_swallowing,           //7 Turning to have bottle in front and swallowing bottle
  s_turning,              //8 Turns a given angle

  s_panic             // Panic state, when something unexpected happened
};

// s_wander
#define WANDER_SPEED 0.4f
#define WANDER_TURNING_SPEED 0.3f
#define WANDER_TURN_DURATION 3000
#define WANDER_STOPPING_THRESHOLD 0.001
#define WANDER_TOF_LEFT_THRESHOLD 24
#define WANDER_TOF_CENTER_THRESHOLD 42
#define WANDER_TOF_RIGHT_THRESHOLD 24
#define WANDER_PROXIMITY_THRESHOLD 150
#define WANDER_PROXIMITY_FRONT_THRESHOLD 300

// s_following
#define FOLLOWING_WALL_MAX_SPEED 0.6f
#define FOLLOWING_WALL_MIN_SPEED 0.1f
#define FOLLOWING_WALL_MAX_SPEED_ANGLE 90.f
#define FOLLOWING_WALL_APPROACHING_FACTOR 0.6f
#define FOLLOWING_WALL_RIGHT 0
#define FOLLOWING_WALL_LEFT 0
#define FOLLOWING_WALL_CORNER_DETECTED_THRESHOLD 150
#define FOLLOWING_WALL_DESIRED_WALL_DISTANCE 80
#define FOLLOWING_WALL_REACTIVITY 0.002
#define FOLLOWING_WALL_STOPPING_THRESHOLD 0.001

// s_swallowing
#define SWALLOWING_TURNING_SPEED 0.4f
#define SWALLOWING_TURNING_THRESHOLD -40
#define SWALLOWING_STOPPING_THRESHOLD 0.001
#define SWALLOWING_TOF_THRESHOLD 42
#define SWALLOWING_PROXIMITY_THRESHOLD 80
#define SWALLOWING_SPEED 0.4f
#define SWALLOWING_OPEN_DURATION 1500
#define SWALLOWING_DURATION_OFFSET 500
#define SWALLOWING_DURATION 4000
#define SWALLOWING_BOTTLE_DETECTION_THRESHOLD 60
// s_scanning
#define SCANNING_TURNING_SPEED 0.2f
#define SCANNING_TURNING_THRESHOLD -40
#define SCANNING_STOPPING_THRESHOLD 0.001
#define SCANNING_Z_DELTA_THRESHOLD 0.1f
#define SCANNING_PROX_THRESHOLD 200.f
#define SCANNING_ORIENTING_STOPPING_THRESHOLD 7.5f
#define SCANNING_ORIENTING_REACTIVITY 0.008
#define SCANNING_ORIENTING_MAX_SPEED 0.5f
#define SCANNING_CHECKING_TOF_RIGHT_THRESHOLD 18.f
#define SCANNING_CHECKING_TOF_CENTER_THRESHOLD 24.f
#define SCANNING_CHECKING_TOF_LEFT_THRESHOLD 18.f

#define TURNING_SPEED 0.2f
#define TURNING_ANGLE 90.f
#define TURNING_STOPPING_THRESHOLD 0.001f
#define TURNING_ANGLE_THRESHOLD 4.f



#endif
