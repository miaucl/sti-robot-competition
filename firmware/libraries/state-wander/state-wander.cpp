/*
  state-wander.h - Wander state methods
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
  is_forward,
  is_stopping,
  is_turn_start,
  is_turning,
  is_turn_stopping,
  is_detected_stopping,
  is_detected,
  is_off
};
static IState is_state = is_start;
static long turn_timestamp = 0;


void stateWanderEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Random Wandering");
  #endif

}

void stateWanderRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                        int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                        float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT])
{
  Serial.print("wander(");
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
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
  }

  int proximityForwardRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
  int proximityForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
  int proximityForwardLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
  float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
  float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
  float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);




  // Start moving
  if (is_state == is_start)
  {
    Serial.print("start\t");

    // Set default wandering speed
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = WANDER_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = WANDER_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    // Set moving flag
    is_state = is_forward;
  }
  else if (is_state == is_forward)
  {
    Serial.print("tof: ");
    Serial.print(tofCenter);
    Serial.print("ir: ");
    Serial.print(proximityForwardRight);
    Serial.print(", ");
    Serial.print(proximityForward);
    Serial.print(", ");
    Serial.print(proximityForwardLeft);

    if ((tofCenter < WANDER_TOF_THRESHOLD && tofCenter > 0 ) ||
        proximityForwardRight > WANDER_PROXIMITY_THRESHOLD ||
        proximityForward > WANDER_PROXIMITY_THRESHOLD ||
        proximityForwardLeft > WANDER_PROXIMITY_THRESHOLD)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;
    }
  }
  else if (is_state == is_stopping)
  {
    Serial.print("motor: ");
    Serial.print(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    Serial.print("\t");
    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      if (!(tofCenter < WANDER_TOF_THRESHOLD && tofCenter > 0) && proximityForward > WANDER_PROXIMITY_THRESHOLD)
      {
        is_state = is_detected;
      }

      Serial.print(proximityForward);
      Serial.print("\t");
      Serial.print(tofCenter);
      Serial.print("\t");

      is_state = is_turn_start;
    }
  }
  else if (is_state == is_turn_start)
  {
    Serial.print("turn\t");
    // Set default turning speed
    if (millis() % 2 == 0)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = WANDER_TURNING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -WANDER_TURNING_SPEED;
    }
    else
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -WANDER_TURNING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = WANDER_TURNING_SPEED;
    }
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    turn_timestamp = millis();

    is_state = is_turning;
  }
  else if (is_state == is_turning)
  {
    Serial.print("time: ");
    Serial.print(millis() - turn_timestamp);
    Serial.print("\t");
    if (!(tofCenter < WANDER_TOF_THRESHOLD && tofCenter > 0) && proximityForward > WANDER_PROXIMITY_THRESHOLD)
    {
      Serial.print(proximityForward);
      Serial.print("\t");
      Serial.print(tofCenter);
      Serial.print("\t");

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_detected_stopping;
    }
    else if (millis() - turn_timestamp > WANDER_TURN_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_turn_stopping;
    }
  }
  else if (is_state == is_turn_stopping)
  {
    Serial.print("motor: ");
    Serial.print(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    Serial.print("\t");
    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      is_state = is_start;
    }
  }
  else if (is_state == is_detected_stopping)
  {
    Serial.print("motor: ");
    Serial.print(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    Serial.print("\t");
    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      is_state = is_detected;
    }
  }
  else if (is_state == is_detected)
  {
    Serial.print("detected ");
    Serial.print("\t");
  }



  Serial.println("");
}


void stateWanderExitRoutine(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
