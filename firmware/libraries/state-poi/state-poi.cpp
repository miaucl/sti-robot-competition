/*
  state-poi.h - Go to a point of interest
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"
#include "math.h"
#include "utils.h"

#include "Arduino.h"
#include "math.h"
#include "config.h"
#include "sensors.h"
#include "actuators.h"

#include "BasicLinearAlgebra.h"
using namespace BLA;


// Internal control flags
enum IState
{
  is_start,
  is_turn_start,
  is_turning,
  is_turn_stopping,
  is_braitenberg,
  is_off
};

static float pois[2][2] = {{4.f, 4.f}, {6.f, 1.f}};

static IState is_state = is_start;
static int poiIndex = 1;
static float z = 0;
static float error = 0;

static long braitenbergTimestamp = 0;

static int directionalSpeedCount = 0;


void statePOIEnterRoutine(boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  directionalSpeedCount = 0;

  is_state = is_start;
}

void statePOIRoutine( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                      int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                      int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                      int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                      float estimatedAngle,
                      Matrix<2> estimatedPos,
                      double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                      double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                      boolean btnState[BTN_COUNT],
                      boolean ledState[LED_COUNT],
                      boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("poi(");
  Serial.print(is_state);
  Serial.print(")");
  Serial.print("\testimated angle: ");
  Serial.print(estimatedAngle);
  Serial.print("\testimated pos: ");
  Serial.print(estimatedPos(0));
  Serial.print(", ");
  Serial.print(estimatedPos(1));
  Serial.print("\t ");
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

  // Start TURNING
  if (is_state == is_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start turning");
    #endif

    is_state = is_turn_start;
  }

  else if (is_state == is_turn_start)
  {
    float dx = pois[poiIndex][0] - estimatedPos(0);
    float dy = pois[poiIndex][1] - estimatedPos(1);
    float desiredAngle = atan2(dy, dx) / M_PI * 180.f;

    #ifdef SERIAL_ENABLE
    Serial.print("desired angle: ");
    Serial.print(desiredAngle);
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


    is_state = is_turning;

  }
  else if (is_state == is_turning)
  {
    float dx = pois[poiIndex][0] - estimatedPos(0);
    float dy = pois[poiIndex][1] - estimatedPos(1);
    float desiredAngle = atan2(dy, dx) / M_PI * 180.f;


    #ifdef SERIAL_ENABLE
    Serial.print("delta: ");
    Serial.print(dx);
    Serial.print(", ");
    Serial.print(dy);
    Serial.print("\tdesired angle: ");
    Serial.print(desiredAngle);
    Serial.print(" \t ");
    #endif

    float error = wrapPI(estimatedAngle - desiredAngle);
    float turningSpeed = error * POI_TURNING_REACTIVITY;
    turningSpeed = max(turningSpeed, -POI_TURNING_MAX_SPEED);
    turningSpeed = min(turningSpeed, POI_TURNING_MAX_SPEED);
    #ifdef SERIAL_ENABLE
    Serial.print("turning: ");
    Serial.print(error);
    Serial.print(", ");
    Serial.print(turningSpeed);
    #endif

    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -turningSpeed;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = turningSpeed;
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    if (fabsf(error) < POI_STOPPING_THRESHOLD)
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

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < POI_TURNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < POI_TURNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" stopped");
      #endif

      braitenbergTimestamp = millis();

      is_state = is_braitenberg;
    }
  }
  else if (is_state == is_braitenberg)
  {
    float dx = pois[poiIndex][0] - estimatedPos(0);
    float dy = pois[poiIndex][1] - estimatedPos(1);
    float desiredAngle = atan2(dy, dx) / M_PI * 180.f;

    float rightSpeed = 0;
    float leftSpeed = 0;

    // Bias
    rightSpeed += POI_BRAITENBERG_BIAS;
    leftSpeed += POI_BRAITENBERG_BIAS;

    // Right tof sensor
    float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
    if (tofRight > 0)
    {
      rightSpeed += POI_BRAITENBERG_TOF_RIGHT_RIGHT_FACTOR * (POI_BRAITENBERG_TOF_MAX - tofRight);
      leftSpeed += POI_BRAITENBERG_TOF_RIGHT_LEFT_FACTOR * (POI_BRAITENBERG_TOF_MAX - tofRight);
    }

    // Forward tof sensor
    float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    if (tofCenter > 0)
    {
      rightSpeed += POI_BRAITENBERG_TOF_FORWARD_FACTOR * (POI_BRAITENBERG_TOF_MAX - tofCenter);
      leftSpeed += POI_BRAITENBERG_TOF_FORWARD_FACTOR * (POI_BRAITENBERG_TOF_MAX - tofCenter);
    }

    // Left tof sensor
    float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);
    if (tofLeft > 0)
    {
      rightSpeed += POI_BRAITENBERG_TOF_LEFT_RIGHT_FACTOR * (POI_BRAITENBERG_TOF_MAX - tofLeft);
      leftSpeed += POI_BRAITENBERG_TOF_LEFT_LEFT_FACTOR * (POI_BRAITENBERG_TOF_MAX - tofLeft);
    }


    // Forward prox sensor
    int proxForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
    rightSpeed += POI_BRAITENBERG_PROX_FORWARD_FACTOR * proxForward;
    leftSpeed += POI_BRAITENBERG_PROX_FORWARD_FACTOR * proxForward;

    // Forward prox left sensor
    int proxLeftForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
    rightSpeed += POI_BRAITENBERG_PROX_FORWARD_LEFT_RIGHT_FACTOR * abs(proxLeftForward);
    leftSpeed += POI_BRAITENBERG_PROX_FORWARD_LEFT_LEFT_FACTOR * abs(proxLeftForward);

    // Forward prox right sensor
    int proxRightForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
    rightSpeed += POI_BRAITENBERG_PROX_FORWARD_RIGHT_RIGHT_FACTOR * abs(proxRightForward);
    leftSpeed += POI_BRAITENBERG_PROX_FORWARD_RIGHT_LEFT_FACTOR * abs(proxRightForward);


    // Prox left sensor
    int proxLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT];
    rightSpeed += POI_BRAITENBERG_PROX_LEFT_RIGHT_FACTOR * abs(proxLeft);
    leftSpeed += POI_BRAITENBERG_PROX_LEFT_LEFT_FACTOR * abs(proxLeft);

    // Prox right sensor
    int proxRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT];
    rightSpeed += POI_BRAITENBERG_PROX_RIGHT_RIGHT_FACTOR * abs(proxRight);
    leftSpeed += POI_BRAITENBERG_PROX_RIGHT_LEFT_FACTOR * abs(proxRight);

    // Angle
    float angleError = wrapPI(estimatedAngle - desiredAngle);
    rightSpeed -= POI_BRAITENBERG_ANGLE_ERROR_FACTOR * angleError;
    leftSpeed += POI_BRAITENBERG_ANGLE_ERROR_FACTOR * angleError;

    rightSpeed = max(rightSpeed, -POI_BRAITENBERG_MAX_SPEED);
    rightSpeed = min(rightSpeed, POI_BRAITENBERG_MAX_SPEED);
    leftSpeed = max(leftSpeed, -POI_BRAITENBERG_MAX_SPEED);
    leftSpeed = min(leftSpeed, POI_BRAITENBERG_MAX_SPEED);
    #ifdef SERIAL_ENABLE
    Serial.print("braitenberg: ");
    Serial.print(leftSpeed);
    Serial.print(", ");
    Serial.print(rightSpeed);
    #endif

    // #ifdef SERIAL_ENABLE
    // Serial.print("prox: ");
    // Serial.print(proxLeftForward);
    // Serial.print(", ");
    // Serial.print(proxForward);
    // Serial.print(", ");
    // Serial.print(proxRightForward);
    // Serial.print(", ");
    // Serial.print(tofLeft);
    // Serial.print(", ");
    // Serial.print(tofCenter);
    // Serial.print(", ");
    // Serial.print(tofRight);
    // Serial.print(", ");
    // Serial.print(angleError);
    // Serial.print(" => ");
    // #endif


    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = rightSpeed;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = leftSpeed;
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    // Target reached
    if (fabs(dx) + fabs(dy) < POI_BRAITENBERG_TARGET_REACHED_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > reached");
      #endif
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      flags[FLAG_POI_REACHED] = 1;

      is_state = is_off;
    }
    // No directional speed
    else if (millis() - braitenbergTimestamp > POI_BRAITENBERG_THRESHOLD &&
            fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < POI_BRAITENBERG_STOPPING_THRESHOLD &&
            fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < POI_BRAITENBERG_STOPPING_THRESHOLD)
    {
      directionalSpeedCount++;

      if (directionalSpeedCount > POI_BRAITENBERG_STOPPING_COUNT)
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > stopping");
        #endif
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

        flags[FLAG_POI_REACHED] = 1;

        is_state = is_off;
      }
    }
    else
    {
      directionalSpeedCount = 0;
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void statePOIExitRoutine( boolean ledState[LED_COUNT],
                          boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
