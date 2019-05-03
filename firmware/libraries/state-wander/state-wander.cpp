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

static boolean ignoreRight = false;
static boolean ignoreLeft = false;


void stateWanderEnterRoutine( boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;
}

void stateWanderRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
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
  Serial.print("wander(");
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
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
  }
  #endif

  int proximityRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT];
  int proximityForwardRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
  int proximityForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
  int proximityForwardLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
  int proximityLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT];
  float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
  float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
  float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);




  // Start moving
  if (is_state == is_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start\t");
    #endif
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
    #ifdef SERIAL_ENABLE
    Serial.print("tof: ");
    Serial.print(tofLeft);
    Serial.print(", ");
    Serial.print(tofCenter);
    Serial.print(", ");
    Serial.print(tofRight);
    Serial.print(" ir: ");
    Serial.print(proximityRight);
    Serial.print(", ");
    Serial.print(proximityForwardRight);
    Serial.print(", ");
    Serial.print(proximityForward);
    Serial.print(", ");
    Serial.print(proximityForwardLeft);
    Serial.print(", ");
    Serial.print(proximityLeft);
    #endif

    if (ignoreRight)
    {
      if ((proximityRight > WANDER_PROXIMITY_THRESHOLD) ||
      (proximityForwardRight > WANDER_PROXIMITY_THRESHOLD))
      {
        proximityRight = 0;
        proximityForwardRight = 0;
      }
      else
      {
        ignoreRight = false;
      }
    }

    if (ignoreLeft)
    {
      if ((proximityLeft > WANDER_PROXIMITY_THRESHOLD) ||
      (proximityForwardLeft > WANDER_PROXIMITY_THRESHOLD))
      {
        proximityLeft = 0;
        proximityForwardLeft = 0;
      }
      else
      {
        ignoreLeft = false;
      }
    }

    if (proximityRight > WANDER_PROXIMITY_THRESHOLD ||
        proximityForwardRight > WANDER_PROXIMITY_THRESHOLD ||
        proximityForwardLeft > WANDER_PROXIMITY_THRESHOLD ||
        proximityLeft > WANDER_PROXIMITY_THRESHOLD)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_detected_stopping;
    }
    else if  ((tofLeft < WANDER_TOF_LEFT_THRESHOLD && tofLeft > 0) ||
              (tofCenter < WANDER_TOF_CENTER_THRESHOLD && tofCenter > 0) ||
              (tofRight < WANDER_TOF_RIGHT_THRESHOLD && tofRight > 0) ||
              (proximityForward > WANDER_PROXIMITY_THRESHOLD))
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
    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      if ((tofLeft > WANDER_TOF_LEFT_THRESHOLD || tofLeft == 0) &&
          (tofCenter > WANDER_TOF_CENTER_THRESHOLD || tofCenter == 0) &&
          (tofRight > WANDER_TOF_RIGHT_THRESHOLD || tofRight == 0) &&
          proximityForward > WANDER_PROXIMITY_THRESHOLD)
      {
        #ifdef SERIAL_ENABLE
        Serial.print("prox detection");
        #endif

        is_state = is_detected;
      }
      else
      {
        #ifdef SERIAL_ENABLE
        Serial.print("tof detection");
        #endif
        is_state = is_turn_start;
      }
    }
  }
  else if (is_state == is_turn_start)
  {
    boolean turnLeft;
    if (tofRight > 0) turnLeft = false;
    else if (tofLeft > 0) turnLeft = true;
    else turnLeft = millis() % 2 == 0;

    #ifdef SERIAL_ENABLE
    Serial.print("start turn");
    #endif
    // Set default turning speed
    if (turnLeft)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = WANDER_TURNING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -WANDER_TURNING_SPEED;
      ignoreRight = true;
    }
    else
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -WANDER_TURNING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = WANDER_TURNING_SPEED;
      ignoreLeft = true;
    }
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    turn_timestamp = millis();

    is_state = is_turning;
  }
  else if (is_state == is_turning)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("time: ");
    Serial.print(millis() - turn_timestamp);
    #endif

    if ((tofLeft > WANDER_TOF_LEFT_THRESHOLD || tofLeft == 0) &&
        (tofCenter > WANDER_TOF_CENTER_THRESHOLD || tofCenter == 0) &&
        (tofRight > WANDER_TOF_RIGHT_THRESHOLD || tofRight == 0) &&
        proximityForward > WANDER_PROXIMITY_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > stopping");
      #endif

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
    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("turn stopped");
      #endif

      is_state = is_start;
    }
  }
  else if (is_state == is_detected_stopping)
  {
    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("stopped");
      #endif

      is_state = is_detected;
    }
  }
  else if (is_state == is_detected)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("detected");
    #endif

  }



  Serial.println("");
}


void stateWanderExitRoutine(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
