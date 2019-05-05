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
static int bottleInRobot = 0;
static boolean zOverflow = false;
static float zStart = 0;
static float zLast = 0;

static int scanMaxProxRight = 0;
static int scanMaxAngleRight = 0;
static int scanMaxProxForwardRight = 0;
static int scanMaxAngleForwardRight = 0;
static int scanMaxProxForward = 0;
static int scanMaxAngleForward = 0;
static int scanMaxProxLeft = 0;
static int scanMaxAngleLeft = 0;
static int scanMaxProxForwardLeft = 0;
static int scanMaxAngleForwardLeft = 0;
static int count = 0;

static int targetAngle = 0;


void stateScanningEnterRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  is_state = is_start;

  maxProx = 0;
  maxProxVal = 0;
  beforeProxVal = 0;
  scanningTimestamp = 0;

  scanMaxProxRight = 0;
  scanMaxAngleRight = 0;
  scanMaxProxForwardRight = 0;
  scanMaxAngleForwardRight = 0;
  scanMaxProxForward = 0;
  scanMaxAngleForward = 0;
  scanMaxProxLeft = 0;
  scanMaxAngleLeft = 0;
  scanMaxProxForwardLeft = 0;
  scanMaxAngleForwardLeft = 0;
  count = 0;

  targetAngle = 0;

  zOverflow = false;

  bottleInRobot = 0;
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
  #ifdef SERIAL_ENABLE
  Serial.print("scanning(");
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

  // Start scanning bottle
  if (is_state == is_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start");
    #endif

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

    #ifdef SERIAL_ENABLE
    Serial.print("start angle: ");
    Serial.print(zStart);
    #endif
  }
  else if (is_state == is_turning)
  {
    float z = getMedianIMUZOrientationValue(imuMeasurements);

    if (fabsf(z - zLast) > SCANNING_Z_DELTA_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("angle: ");
      Serial.print(z);
      #endif

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

      if (scanMaxProxRight < proxRight && SCANNING_PROX_THRESHOLD < proxRight)
      {
        scanMaxProxRight = proxRight;
        scanMaxAngleRight = z + SENSOR_PROXIMITY_RIGHT_OFFSET;
        if (scanMaxAngleRight > 180) scanMaxAngleRight -= 360;
        if (scanMaxAngleRight < -180) scanMaxAngleRight += 360;
      }
      if (scanMaxProxForwardRight < proxForwardRight && SCANNING_PROX_THRESHOLD < proxForwardRight)
      {
        scanMaxProxForwardRight = proxForwardRight;
        scanMaxAngleForwardRight = z + SENSOR_PROXIMITY_FORWARD_RIGHT_OFFSET;
        if (scanMaxAngleForwardRight > 180) scanMaxAngleForwardRight -= 360;
        if (scanMaxAngleForwardRight < -180) scanMaxAngleForwardRight += 360;
      }
      if (scanMaxProxForward < proxForward && SCANNING_PROX_THRESHOLD < proxForward)
      {
        scanMaxProxForward = proxForward;
        scanMaxAngleForward = z + SENSOR_PROXIMITY_FORWARD_OFFSET;
        if (scanMaxAngleForward > 180) scanMaxAngleForward -= 360;
        if (scanMaxAngleForward < -180) scanMaxAngleForward += 360;
      }
      if (scanMaxProxForwardLeft < proxForwardLeft && SCANNING_PROX_THRESHOLD < proxForwardLeft)
      {
        scanMaxProxForwardLeft = proxForwardLeft;
        scanMaxAngleForwardLeft = z + SENSOR_PROXIMITY_FORWARD_LEFT_OFFSET;
        if (scanMaxAngleForwardLeft > 180) scanMaxAngleForwardLeft -= 360;
        if (scanMaxAngleForwardLeft < -180) scanMaxAngleForwardLeft += 360;
      }
      if (scanMaxProxLeft < proxLeft && SCANNING_PROX_THRESHOLD < proxLeft)
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
        #ifdef SERIAL_ENABLE
        Serial.print("\tMAX Right: ");
        Serial.print(scanMaxProxRight);
        Serial.print(",");
        Serial.print(scanMaxAngleRight);
        Serial.print("\tMAX Forward Right: ");
        Serial.print(scanMaxProxLeft);
        Serial.print(",");
        Serial.print(scanMaxAngleForwardRight);
        Serial.print("\tMAX Forward: ");
        Serial.print(scanMaxProxForward);
        Serial.print(",");
        Serial.print(scanMaxAngleForward);
        Serial.print("\tMAX Forward Left: ");
        Serial.print(scanMaxProxForwardLeft);
        Serial.print(",");
        Serial.print(scanMaxAngleForwardLeft);
        Serial.print("\tMAX Left: ");
        Serial.print(scanMaxProxLeft);
        Serial.print(",");
        Serial.print(scanMaxAngleLeft);
        Serial.print("\t\t\t");
        #endif

        targetAngle = 0;
        // Care for circularity of values (-pi,pi)
        if (fabsf(targetAngle - scanMaxAngleForward) < 180 && scanMaxProxForward > 0)
        {
          targetAngle += scanMaxAngleForward;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleRight) < 180 && scanMaxProxRight > 0)
        {
          targetAngle += scanMaxAngleRight;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleForwardRight) < 180 && scanMaxProxForwardRight > 0)
        {
          targetAngle += scanMaxAngleForwardRight;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleForward) < 180 && scanMaxProxForwardLeft > 0)
        {
          targetAngle += scanMaxAngleForward;
          count++;
        }
        if (fabsf(targetAngle - scanMaxAngleLeft) < 180 && scanMaxProxLeft > 0)
        {
          targetAngle += scanMaxAngleLeft;
          count++;
        }

        if (count > 0)
        {
          targetAngle /= count;

          #ifdef SERIAL_ENABLE
          Serial.print(" ==> target angle: ");
          Serial.print(targetAngle);
          #endif


          // Stop motors
          motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
          motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

          is_state = is_turn_stopping;
        }
        else
        {
          #ifdef SERIAL_ENABLE
          Serial.print("nothing detected");
          #endif

          flags[FLAG_NOTHING_DETECTED] = 1;

          is_state = is_off;
        }
      }
    }
  }
  else if (is_state == is_turn_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < SCANNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < SCANNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("scanning stopped");
      #endif

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
    #ifdef SERIAL_ENABLE
    Serial.print("orienting: ");
    Serial.print(error);
    Serial.print(", ");
    Serial.print(turningSpeed);
    #endif

    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = turningSpeed;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = -turningSpeed;
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    if (fabsf(error) < SCANNING_ORIENTING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > stopping");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_checking;
    }
  }
  else if (is_state == is_checking)
  {
    float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
    if ((tofLeft > 0 && tofLeft < SCANNING_CHECKING_TOF_RIGHT_THRESHOLD) ||
        (tofCenter > 0 && tofCenter < SCANNING_CHECKING_TOF_CENTER_THRESHOLD) ||
        (tofRight > 0 && tofRight < SCANNING_CHECKING_TOF_LEFT_THRESHOLD))
    {
      #ifdef SERIAL_ENABLE
      Serial.print("obstacle detected");
      #endif

      flags[FLAG_WALL_DETECTED] = 1;

      is_state = is_off;
    }
    else
    {
      #ifdef SERIAL_ENABLE
      Serial.print("bottle detected");
      #endif

      flags[FLAG_BOTTLE_DETECTED] = 1;

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
