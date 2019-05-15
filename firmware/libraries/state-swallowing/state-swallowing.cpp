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
  is_bottle_swallowed,
  is_off
};
static IState is_state = is_start;
static long openTimestamp = 0;
static long swallowingTimestamp = 0;
static int proxDetect = 0;
static int bottleInRobot = 0;
static int proxDetectZero = 0;


void stateSwallowingEnterRoutine( boolean ledState[LED_COUNT],
                                  boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  is_state = is_start;
}

void stateSwallowingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                            int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                            int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                            double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                            int servoAngles[ACTUATOR_SERVO_COUNT],
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("swallowing(");
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

  // Start swallowing bottle
  if (is_state == is_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("open");
    #endif

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
    if (millis() - openTimestamp > SWALLOWING_OPEN_DURATION)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("opened");
      #endif

      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = SWALLOWING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = SWALLOWING_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      swallowingTimestamp = millis();
      proxDetectZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT];
      is_state = is_move;
    }
  }

  else if (is_state == is_move)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("move \ttime: ");
    Serial.print(millis() - swallowingTimestamp);
    Serial.print("\t");
    #endif

    proxDetect = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT];

    #ifdef SERIAL_ENABLE
    Serial.print("\tbottle detection: ");
    Serial.print(proxDetectZero);
    Serial.print(", ");
    Serial.print(proxDetect);
    Serial.print("\t");
    #endif

    if (millis() - swallowingTimestamp > SWALLOWING_DURATION)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > timeout");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;
    }
    else if (millis() - swallowingTimestamp > SWALLOWING_DURATION_OFFSET && fabsf(proxDetect - proxDetectZero) > SWALLOWING_BOTTLE_DETECTION_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > detection");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      bottleInRobot = 1;
      is_state = is_stopping;
    }
  }

  else if (is_state == is_stopping)
  {
    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < SWALLOWING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < SWALLOWING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("stopped and close");
      #endif

      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

      if(bottleInRobot == 1)
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > bottle swallowed");
        #endif

        flags[FLAG_SWALLOWED_BOTTLE] = 1;

        is_state = is_bottle_swallowed;
      }
      else
      {
        flags[FLAG_SWALLOW_TIMEOUT] = 1;

        is_state = is_off;
      }
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateSwallowingExitRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  resetServoAngle(ACTUATOR_SERVO_BAR_RIGHT_CLOSED, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  resetServoAngle(ACTUATOR_SERVO_BAR_LEFT_CLOSED, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
}
