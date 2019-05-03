/*
  state-analysis.h - Following state methods
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "math.h"
#include "config.h"
#include "sensors.h"
#include "actuators.h"

// Internal control flags
enum IState
{
  is_start,
  //is_calibrating,
  is_following_wall,
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

void stateFollowingEnterRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;
}

void stateFollowingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                          int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                          int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                          int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("following(");
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
    #ifdef SERIAL_ENABLE
    Serial.print("start\t");
    #endif

    // determine if wall on the left or right is to follow
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) > getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) && false)
    {
      #ifdef SERIAL_ENABLE
      Serial.println("> wall: right");
      #endif

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
      Serial.println("> wall: right");
      #endif

      followingWallLeft = 1;
      proximityToConsider = SENSOR_PROXIMITY_FORWARD_LEFT;
      wallNearMotor = ACTUATOR_MOTOR_LEFT;
      wallNearMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
      wallNearMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;

      wallFarMotor = ACTUATOR_MOTOR_RIGHT;
      wallFarMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
      wallFarMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;
    }

    // Set default calibrating speed
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = FOLLOWING_WALL_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = FOLLOWING_WALL_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    // Set moving flag
    is_state = is_following_wall;
  }

  else if (is_state == is_following_wall)
  {
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] > FOLLOWING_WALL_CORNER_DETECTED_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.println("corner detected");
      #endif

       // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;
    }
    else
    {
      int prox = getAverageProximityValue(proximityMeasurements, proximityToConsider) - proximityAmbientMeasurements[proximityToConsider];
      float error = (prox - FOLLOWING_WALL_DESIRED_WALL_DISTANCE);

      // Adjust motor speed
      motorSpeeds[wallNearMotor] = FOLLOWING_WALL_SPEED + FOLLOWING_WALL_REACTIVITY * error;
      motorSpeeds[wallFarMotor] = FOLLOWING_WALL_SPEED - FOLLOWING_WALL_REACTIVITY * error;

      #ifdef SERIAL_ENABLE
      Serial.println("following: ");
      Serial.print("\tprox: ");
      Serial.print(prox);
      Serial.print("\terror: ");
      Serial.print(error);
      Serial.print("\tspeed: ");
      Serial.print(motorSpeeds[wallNearMotor]);
      Serial.print(", ");
      Serial.print(motorSpeeds[wallFarMotor]);
      #endif

      writeRawMotorSpeed(motorSpeeds, wallNearMotor, wallNearMotorDirectionPin, wallNearMotorSpeedPin);
      writeRawMotorSpeed(motorSpeeds, wallFarMotor, wallFarMotorDirectionPin, wallFarMotorSpeedPin);
    }

  }

  else if (is_state == is_stopping)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < FOLLOWING_WALL_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < FOLLOWING_WALL_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("stopped");
      #endif

      is_state = is_off;
    }
  }

}


void stateFollowingExitRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
