/*
  state-slope-down.h - Slope down state methods
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "math.h"
#include "utils.h"
#include "config.h"
#include "sensors.h"
#include "actuators.h"

// Internal control flags
enum IState
{
  is_start,
  is_following_wall,
  is_stopping_in_front_of_slope,
  is_open,
  is_move_back,
  is_move_back_stopping,
  is_move_forward,
  is_move_forward_stopping,
  is_stopping,
  is_off
};
static IState is_state = is_start;
static int followingWallRight = 0;
static int followingWallLeft = 0;
static int proximityToConsider = 0;
static int wallNearMotor = 0;
static int wallNearMotorDirectionPin = 0;
static int wallNearMotorSpeedPin = 0;
static int wallFarMotor = 0;
static int wallFarMotorDirectionPin = 0;
static int wallFarMotorSpeedPin = 0;

static float proxDownLeftZero = 0;
static float proxDownRightZero = 0;

static boolean bottleDelivered = 0;

static long timestamp = 0;

void stateSlopeDownEnterRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;
  bottleDelivered = 0;

  is_state = is_start;
}

void stateSlopeDownRoutine( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                            int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                            int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                            int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                            float estimatedAngle,
                            float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                            double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                            int servoAngles[ACTUATOR_SERVO_COUNT],
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("slope-down(");
  Serial.print(is_state);
  Serial.print(")\t");
  #endif

  #ifdef DEBUG_ENABLE
  if (Serial.available() > 0)
  {
    char b = Serial.read();
    if (b == ' ')
    {
      stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
      is_state = is_off;
    }
    else if (b == 's')
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
  }
  #endif

  // Start following wall
  if (is_state == is_start)
  {
    int proxLookLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT];
    int proxLookRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT];

    if (proxLookLeft > 0 || proxLookRight > 0)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("start:\t");
      Serial.print(proxLookLeft);
      Serial.print(" / ");
      Serial.print(proxLookRight);
      #endif

      // determine if wall on the left or right is to follow
      if (proxLookRight > proxLookLeft)
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > wall: right");
        #endif

        flags[FLAG_FOLLOWING_RIGHT_SIDE] = 1;

        followingWallRight = 1;
        proximityToConsider = SENSOR_PROXIMITY_FORWARD_RIGHT;
        wallNearMotor = ACTUATOR_MOTOR_RIGHT;
        wallNearMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
        wallNearMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;

        wallFarMotor = ACTUATOR_MOTOR_LEFT;
        wallFarMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
        wallFarMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;
      }
      else
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > wall: left");
        #endif

        flags[FLAG_FOLLOWING_RIGHT_SIDE] = 0;

        followingWallLeft = 1;
        proximityToConsider = SENSOR_PROXIMITY_FORWARD_LEFT;
        wallNearMotor = ACTUATOR_MOTOR_LEFT;
        wallNearMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
        wallNearMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;

        wallFarMotor = ACTUATOR_MOTOR_RIGHT;
        wallFarMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
        wallFarMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;
      }

      // Set following speed to 0
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      // Get angles
      proxDownLeftZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT];
      proxDownRightZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT];


      // Set moving flag
      is_state = is_following_wall;
    }
  }

  else if (is_state == is_following_wall)
  {
    float proxDownLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT] - proxDownLeftZero;
    float proxDownRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT] - proxDownRightZero;

    #ifdef SERIAL_ENABLE
    Serial.print("\tprox d: ");
    Serial.print(proxDownLeft);
    Serial.print(", ");
    Serial.print(proxDownRight);
    #endif



    if (-proxDownLeft > SLOPE_DOWN_PROXIMITY_DOWN_THRESHOLD || -proxDownRight > SLOPE_DOWN_PROXIMITY_DOWN_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("slope detected");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping_in_front_of_slope;
    }
    else
    {
      int prox = getAverageProximityValue(proximityMeasurements, proximityToConsider) - proximityAmbientMeasurements[proximityToConsider];
      float error = (prox - SLOPE_DOWN_DESIRED_WALL_DISTANCE);

      int tof = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);

      // Go fast when low error, else turn first
      float directionalSpeed = SLOPE_DOWN_MAX_SPEED * (SLOPE_DOWN_MAX_SPEED_ANGLE - fabsf(error)) / SLOPE_DOWN_MAX_SPEED_ANGLE;
      if (directionalSpeed < SLOPE_DOWN_MIN_SPEED)
      {
        directionalSpeed = SLOPE_DOWN_MIN_SPEED;
      }
      if (directionalSpeed > SLOPE_DOWN_MAX_SPEED)
      {
        directionalSpeed = SLOPE_DOWN_MAX_SPEED;
      }

      // 0 value passing controller for angle
      float rotationalSpeed = SLOPE_DOWN_REACTIVITY * error;

      // Adjust motor speed
      motorSpeeds[wallNearMotor] = directionalSpeed + rotationalSpeed;
      motorSpeeds[wallFarMotor] = directionalSpeed - rotationalSpeed;

      #ifdef SERIAL_ENABLE
      Serial.print("\tprox: ");
      Serial.print(prox);
      Serial.print("\terror: ");
      Serial.print(error);
      Serial.print("\ttof: ");
      Serial.print(tof);
      Serial.print("\tspeed: ");
      Serial.print(motorSpeeds[wallNearMotor]);
      Serial.print(", ");
      Serial.print(motorSpeeds[wallFarMotor]);
      #endif

      writeRawMotorSpeed(motorSpeeds, wallNearMotor, wallNearMotorDirectionPin, wallNearMotorSpeedPin);
      writeRawMotorSpeed(motorSpeeds, wallFarMotor, wallFarMotorDirectionPin, wallFarMotorSpeedPin);
    }
  }
  else if (is_state == is_stopping_in_front_of_slope)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < SLOPE_DOWN_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < SLOPE_DOWN_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("slope stopped");
      #endif

      is_state = is_open;
    }
  }
  else if (is_state == is_open)
  {
    servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
    servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;
    writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
    writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -SLOPE_DOWN_BACK_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = -SLOPE_DOWN_BACK_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    timestamp = millis();

    is_state = is_move_back;
  }
  else if (is_state == is_move_back)
  {
    if (millis() - timestamp > ((bottleDelivered) ? SLOPE_DOWN_BACK_DURATION_DELIVERED : SLOPE_DOWN_BACK_DURATION))
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_move_back_stopping;
    }
  }
  else if (is_state == is_move_back_stopping)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < SLOPE_DOWN_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < SLOPE_DOWN_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("back stopped");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = SLOPE_DOWN_FORWARD_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = SLOPE_DOWN_FORWARD_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      timestamp = millis();

      if (bottleDelivered)
      {
        flags[FLAG_SWALLOWED_BOTTLE] = 0;
        is_state = is_off;
      }
      else
      {
        is_state = is_move_forward;
      }
    }
  }
  else if (is_state == is_move_forward)
  {
    if (millis() - timestamp > SLOPE_DOWN_FORWARD_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_move_forward_stopping;
    }
  }
  else if (is_state == is_move_forward_stopping)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < SLOPE_DOWN_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < SLOPE_DOWN_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("forward stopped");
      #endif

      bottleDelivered = 1;

      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -SLOPE_DOWN_BACK_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -SLOPE_DOWN_BACK_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


      is_state = is_move_back;
    }
  }


  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateSlopeDownExitRoutine(boolean ledState[LED_COUNT],
                               boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  // setServoAngle(ACTUATOR_SERVO_BAR_RIGHT_CLOSED, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  // setServoAngle(ACTUATOR_SERVO_BAR_LEFT_CLOSED, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
}
