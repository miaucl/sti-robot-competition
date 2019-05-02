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
  is_turn_start,
  is_turning,
  is_turn_stopping,

  is_orienting,
  is_checking,

  is_stopping,
  is_off
};
static IState is_state = is_start;
static int maxProx = 0;
static int maxProxVal = 0;
static int beforeProxVal = 0;
static long scanningTimestamp = 0;
static int proxDetect = 0;
static int proxDetectLeft = 0;
static int proxDetectRight = 0;
static int bottleInRobot = 0;
static boolean zOverflow = false;
static float zStart = 0;
static float zLast = 0;

int scanMaxProxRight = 0;
int scanMaxAngleRight = 0;
int scanMaxProxForwardRight = 0;
int scanMaxAngleForwardRight = 0;
int scanMaxProxForward = 0;
int scanMaxAngleForward = 0;
int scanMaxProxLeft = 0;
int scanMaxAngleLeft = 0;
int scanMaxProxForwardLeft = 0;
int scanMaxAngleForwardLeft = 0;

int targetAngle = 0;

int checkingCounter = 0;


void stateScanningEnterRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Scanning Bottle");
  #endif
}

void stateScanningRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
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
  Serial.print("Scanning bottle(");
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

  // Start scanning bottle
  if (is_state == is_start)
  {
    is_state = is_turn_start;
  }

  else if (is_state == is_turn_start)
  {
    // Set default turning speed (always turn right/left)
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -SCANNING_TURNING_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = SCANNING_TURNING_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    zOverflow = false;
    zStart = getMedianIMUZOrientationValue(imuMeasurements);
    zLast = zStart;
    is_state = is_turning;
  }
  else if (is_state == is_turning)
  {
    float z = getMedianIMUZOrientationValue(imuMeasurements);
    if (fabsf(z - zLast) > SCANNING_Z_DELTA_THRESHOLD)
    {
      int proxRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT];
      int proxForwardRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
      int proxForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
      int proxForwardLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
      int proxLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT];

      // Overflow
      if (fabsf(z - zLast) > 180.f)
      {
        zOverflow = true;
      }

      // Serial.print(z);
      // Serial.print(",");
      // Serial.print(proxRight);
      // Serial.print(",");
      // Serial.print(proxForwardRight);
      // Serial.print(",");
      // Serial.print(proxForward);
      // Serial.print(",");
      // Serial.print(proxForwardLeft);
      // Serial.print(",");
      // Serial.print(proxLeft);
      // // Serial.print(",");
      // Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER));
      // Serial.println();

      if (scanMaxProxRight < proxRight)
      {
        scanMaxProxRight = proxRight;
        scanMaxAngleRight = z + SENSOR_PROXIMITY_RIGHT_OFFSET;
        if (scanMaxAngleRight > 180) scanMaxAngleRight -= 360;
        if (scanMaxAngleRight < -180) scanMaxAngleRight += 360;
      }
      if (scanMaxProxForwardRight < proxForwardRight)
      {
        scanMaxProxForwardRight = proxForwardRight;
        scanMaxAngleForwardRight = z + SENSOR_PROXIMITY_FORWARD_RIGHT_OFFSET;
        if (scanMaxAngleForwardRight > 180) scanMaxAngleForwardRight -= 360;
        if (scanMaxAngleForwardRight < -180) scanMaxAngleForwardRight += 360;
      }
      if (scanMaxProxForward < proxForward)
      {
        scanMaxProxForward = proxForward;
        scanMaxAngleForward = z + SENSOR_PROXIMITY_FORWARD_OFFSET;
        if (scanMaxAngleForward > 180) scanMaxAngleForward -= 360;
        if (scanMaxAngleForward < -180) scanMaxAngleForward += 360;
      }
      if (scanMaxProxForwardLeft < proxForwardLeft)
      {
        scanMaxProxForwardLeft = proxForwardLeft;
        scanMaxAngleForwardLeft = z + SENSOR_PROXIMITY_FORWARD_LEFT_OFFSET;
        if (scanMaxAngleForwardLeft > 180) scanMaxAngleForwardLeft -= 360;
        if (scanMaxAngleForwardLeft < -180) scanMaxAngleForwardLeft += 360;
      }
      if (scanMaxProxLeft < proxLeft)
      {
        scanMaxProxLeft = proxLeft;
        scanMaxAngleLeft = z + SENSOR_PROXIMITY_LEFT_OFFSET;
        if (scanMaxAngleLeft > 180) scanMaxAngleLeft -= 360;
        if (scanMaxAngleLeft < -180) scanMaxAngleLeft += 360;
      }


      zLast = z;

      // One revolution
      if (zOverflow && z > zStart)
      {
        // Serial.print("MAX Right: ");
        // Serial.print(scanMaxProxRight);
        // Serial.print(",");
        // Serial.println(scanMaxAngleRight);
        // Serial.print("MAX Forward Right: ");
        // Serial.print(scanMaxProxLeft);
        // Serial.print(",");
        // Serial.println(scanMaxAngleForwardRight);
        // Serial.print("MAX Forward: ");
        // Serial.print(scanMaxProxForward);
        // Serial.print(",");
        // Serial.println(scanMaxAngleForward);
        // Serial.print("MAX Forward Left: ");
        // Serial.print(scanMaxProxForwardLeft);
        // Serial.print(",");
        // Serial.println(scanMaxAngleForwardLeft);
        // Serial.print("MAX Left: ");
        // Serial.print(scanMaxProxLeft);
        // Serial.print(",");
        // Serial.println(scanMaxAngleLeft);

        int count = 1;
        targetAngle = scanMaxAngleForward;
        // Care for circularity of values (-pi,pi)
        if (fabsf(targetAngle - scanMaxAngleRight) < 180)
        {
          targetAngle += scanMaxAngleRight;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleForwardRight) < 180)
        {
          targetAngle += scanMaxAngleForwardRight;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleForward) < 180)
        {
          targetAngle += scanMaxAngleForward;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleLeft) < 180)
        {
          targetAngle += scanMaxAngleLeft;
          count++;
        }

        targetAngle /= count;

        Serial.print("Target angle: ");
        Serial.print(targetAngle);


        // Stop motors
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

        is_state = is_turn_stopping;
      }
    }
  }
  else if (is_state == is_turn_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < SCANNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < SCANNING_STOPPING_THRESHOLD)
    {
      is_state = is_orienting;
    }
  }
  else if (is_state == is_orienting)
  {
    float z = getMedianIMUZOrientationValue(imuMeasurements);
    float error = z - targetAngle;
    float turningSpeed = error * SCANNING_ORIENTING_REACTIVITY;
    turningSpeed = max(turningSpeed, -SCANNING_ORIENTING_MAX_SPEED);
    turningSpeed = min(turningSpeed, SCANNING_ORIENTING_MAX_SPEED);
    Serial.print(error);
    Serial.print(", ");
    Serial.print(turningSpeed);

    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = turningSpeed;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = -turningSpeed;
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    if (fabsf(error) < SCANNING_ORIENTING_STOPPING_THRESHOLD)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      checkingCounter = 0;
      is_state = is_checking;
    }
  }
  else if (is_state == is_checking)
  {
    Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER));
    Serial.print(", ");
    Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]);

    float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    if ((tofLeft > 0 && tofLeft < SCANNING_CHECKING_TOF_RIGHT_THRESHOLD) ||
        (tofCenter > 0 && tofCenter < SCANNING_CHECKING_TOF_CENTER_THRESHOLD) ||
        (tofRight > 0 && tofRight < SCANNING_CHECKING_TOF_LEFT_THRESHOLD))
    {
      Serial.print("OBASTACLE IN FRONT OF US!");
      is_state = is_off;
    }
    else if (checkingCounter++ > SCANNING_CHECKING_MEASUREMENTS)
    {
      Serial.print("GET THE BOTTLE!");
      is_state = is_off;
    }
  }

  Serial.println();
}


void stateScanningExitRoutine(boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
