/*
  state-wander.h - Wander state methods
  Created by Cyrill Lippuner, 2019.
*/



#include "Arduino.h"
#include "math.h"
#include "config.h"
#include "utils.h"
#include "sensors.h"
#include "actuators.h"

// Internal control flags
enum IState
{
  is_start,
  is_turn_start,
  is_turning,
  is_turn_stopping,
  is_off
};

static IState is_state = is_start;
static float zStart = 0;
static float zDesired = 0;
static float z = 0;
static float error = 0;

void stateTurningEnterRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  is_state = is_start;
}

void stateTurningRoutine(float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("turning(");
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

  // Start TURNING bottle
  if (is_state == is_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start turn ");
    if (flags[FLAG_TURN_RIGHT]) Serial.print("right");
    else Serial.print("left");
    #endif

    is_state = is_turn_start;
  }

  else if (is_state == is_turn_start)
  {

    zStart = getMedianIMUZOrientationValue(imuMeasurements);

    float zDelta = 0;

    if (flags[FLAG_TURN_RIGHT])
    {
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -TURNING_SPEED;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = TURNING_SPEED;
        zDelta = TURNING_ANGLE;
    }
    else
    {
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = TURNING_SPEED;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = -TURNING_SPEED;
        zDelta = -TURNING_ANGLE;
    }

    if (flags[FLAG_TURN_ONE_AND_A_HALF])
    {
      zDelta *= 1.5;
    }
    if (flags[FLAG_TURN_DOUBLE])
    {
      zDelta *= 2;
    }

    zDesired = wrapPI(zStart + zDelta);

    #ifdef SERIAL_ENABLE
    Serial.print(" desired angle: ");
    Serial.print(zDesired);
    #endif
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


    is_state = is_turning;

  }
  else if (is_state == is_turning)
  {
    z = getMedianIMUZOrientationValue(imuMeasurements);

    #ifdef SERIAL_ENABLE
    Serial.print(" angle: ");
    Serial.print(z);
    #endif

    error = z-zDesired;
    if (fabsf(error) < TURNING_ANGLE_THRESHOLD || fabsf(error+360) < TURNING_ANGLE_THRESHOLD || fabsf(error-360) < TURNING_ANGLE_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > stopping");
      #endif
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_turn_stopping;
    }
  }
  else if (is_state == is_turn_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < TURNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < TURNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" stopped");
      #endif

      flags[FLAG_TURN_FINISHED] = 1;

      is_state = is_off;
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateTurningExitRoutine(boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
