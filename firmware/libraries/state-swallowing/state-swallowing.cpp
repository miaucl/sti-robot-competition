/*
  state-analysis.h - Following state methods
  Created by Mirko Indumi, 2019.
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
  is_open,
  is_move,
  is_stopping,
  is_off
};
static IState is_state = is_start;
static long openTimestamp = 0;
static long swallowingTimestamp = 0;
static int proxDetectLeft = 0;
static int proxDetectRight = 0;
static int bottleInRobot = 0;


void stateSwallowingEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Swallowing Bottle");
  #endif
}

void stateSwallowingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                          int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                          int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                          int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                          int servoAngles[ACTUATOR_SERVO_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT])
{
  Serial.print("Swallowing bottle(");
  Serial.print(is_state);
  Serial.print(")\t");
  // DEBUG
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

  // Start swallowing bottle
  if (is_state == is_start)
  {
    servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
    servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;
    writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
    writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

    openTimestamp = millis();
    is_state = is_open;
  }

  // Opening the bar
  if (is_state == is_open)
  {
    Serial.print(millis() - openTimestamp);
    if (millis() - openTimestamp > SWALLOWING_OPEN_DURATION)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = SWALLOWING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = SWALLOWING_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      swallowingTimestamp = millis();
      is_state = is_move;
    }
  }

  else if (is_state == is_move)
  {
    Serial.print("time: ");
    Serial.print(millis() - swallowingTimestamp);
    Serial.print("\t");


    proxDetectRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_RIGHT];
    proxDetectLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_LEFT];

    Serial.print(proxDetectRight);
    Serial.print(",");
    Serial.print(proxDetectLeft);

    if (millis() - swallowingTimestamp > SWALLOWING_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;
      Serial.println("Timeout");
    }
    else if (millis() - swallowingTimestamp > SWALLOWING_DURATION_OFFSET && fabs(proxDetectRight) > SWALLOWING_BOTTLE_DETECTION_THRESHOLD && fabs(proxDetectLeft) > SWALLOWING_BOTTLE_DETECTION_THRESHOLD)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      bottleInRobot = 1;
      is_state = is_stopping;
      Serial.println("Detection");
    }
  }

  else if (is_state == is_stopping)
  {
    Serial.print("motor: ");
    Serial.print(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    Serial.print("\t");
    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < SWALLOWING_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < SWALLOWING_STOPPING_THRESHOLD)
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

      if(bottleInRobot == 1)
      {
        Serial.print("Bottle swallowed!");
          //Exit to return state
      }
      else
      {
          //Exit to wander state
      }
    }
  }


  Serial.println();



}


void stateSwallowingExitRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  resetServoAngle(ACTUATOR_SERVO_BAR_RIGHT_CLOSED, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  resetServoAngle(ACTUATOR_SERVO_BAR_LEFT_CLOSED, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
}
