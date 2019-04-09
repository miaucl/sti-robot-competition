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
static bool moving = 0;


void stateWanderEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Random Wandering");
  #endif

}

void stateWanderRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                        int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                        float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                        boolean ledState[LED_COUNT])
{
  // Start moving
  if (!moving)
  {
    // Set default wandering speed
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = WANDER_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = WANDER_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    // Set moving flag
    moving = 1;
  }

  Serial.println(getMedianTOFValue(tofMeasurements, SENSOR_TOF_CENTER));

  if (checkTOFThreshold(tofMeasurements, SENSOR_TOF_RIGHT) || checkTOFThreshold(tofMeasurements, SENSOR_TOF_CENTER) || checkTOFThreshold(tofMeasurements, SENSOR_TOF_LEFT))
  {
    stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  }


  // Update the motor control
  updateMotorSpeedControl(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN, motorPositionMeasurements);
  updateMotorSpeedControl(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN, motorPositionMeasurements);

}


void stateWanderExitRoutine(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
