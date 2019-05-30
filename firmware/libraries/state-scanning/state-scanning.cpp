/*
  state-analysis.h - Following state methods
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

  is_orienting,
  is_checking,

  is_stopping,
  is_back_off_start,
  is_back_off,
  is_back_off_stopping,

  is_off
};
static IState is_state = is_start;
static int maxProx = 0;
static int maxProxVal = 0;
static int beforeProxVal = 0;
static long scanningTimestamp = 0;
static float zStart = 0;
static float zLast = 0;

static long turnOffsetTimestamp = 0;

static boolean bottleDetected = 0;

static int scanMaxProxRight = 0;
static int scanMaxProxAngleRight = 0;
static int scanMaxProxForwardRight = 0;
static int scanMaxProxAngleForwardRight = 0;
static int scanMaxProxForward = 0;
static int scanMaxProxAngleForward = 0;
static int scanMaxProxLeft = 0;
static int scanMaxProxAngleLeft = 0;
static int scanMaxProxForwardLeft = 0;
static int scanMaxProxAngleForwardLeft = 0;
static int proxCount = 0;

static int proxTargetAngle = 0;

static int scanMinTOFRight = SCANNING_TOF_INF;
static int scanMinTOFAngleRight = 0;
static int scanMinTOFCenter = SCANNING_TOF_INF;
static int scanMinTOFAngleCenter = 0;
static int scanMinTOFLeft = SCANNING_TOF_INF;
static int scanMinTOFAngleLeft = 0;
static int tofCount = 0;

static int tofTargetAngle = 0;

static int targetAngle = 0;

static long back_off_timestamp = 0;



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
  scanMaxProxAngleRight = 0;
  scanMaxProxForwardRight = 0;
  scanMaxProxAngleForwardRight = 0;
  scanMaxProxForward = 0;
  scanMaxProxAngleForward = 0;
  scanMaxProxLeft = 0;
  scanMaxProxAngleLeft = 0;
  scanMaxProxForwardLeft = 0;
  scanMaxProxAngleForwardLeft = 0;
  proxCount = 0;
  proxTargetAngle = 0;

  scanMinTOFRight = SCANNING_TOF_INF;
  scanMinTOFAngleRight = 0;
  scanMinTOFCenter = SCANNING_TOF_INF;
  scanMinTOFAngleCenter = 0;
  scanMinTOFLeft = SCANNING_TOF_INF;
  scanMinTOFAngleLeft = 0;
  tofCount = 0;
  tofTargetAngle = 0;

  bottleDetected = 0;
  targetAngle = 0;
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

    turnOffsetTimestamp = millis();

    is_state = is_turn_start;
  }

  else if (is_state == is_turn_start)
  {
    // Set default turning speed (always turn right/left)
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -SCANNING_TURNING_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = SCANNING_TURNING_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

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
      int proxDownLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT];
      int proxDownRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT];


      int tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
      int tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
      int tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);


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
      // Serial.print(",");
      // Serial.print(proxDownLeft);
      // Serial.print(",");
      // Serial.print(proxDownRight);
      // Serial.print(",");
      // Serial.print(tofRight);
      // Serial.print(",");
      // Serial.print(tofCenter);
      // Serial.print(",");
      // Serial.print(tofLeft);
      // Serial.println();

      if (scanMaxProxRight < proxRight && SCANNING_PROX_THRESHOLD < proxRight)
      {
        scanMaxProxRight = proxRight;
        scanMaxProxAngleRight = wrapPI(z + SENSOR_PROXIMITY_RIGHT_OFFSET);
      }
      if (scanMaxProxForwardRight < proxForwardRight && SCANNING_PROX_THRESHOLD < proxForwardRight)
      {
        scanMaxProxForwardRight = proxForwardRight;
        scanMaxProxAngleForwardRight = wrapPI(z + SENSOR_PROXIMITY_FORWARD_RIGHT_OFFSET);
      }
      if (scanMaxProxForward < proxForward && SCANNING_PROX_THRESHOLD < proxForward)
      {
        scanMaxProxForward = proxForward;
        scanMaxProxAngleForward = wrapPI(z + SENSOR_PROXIMITY_FORWARD_OFFSET);
      }
      if (scanMaxProxForwardLeft < proxForwardLeft && SCANNING_PROX_THRESHOLD < proxForwardLeft)
      {
        scanMaxProxForwardLeft = proxForwardLeft;
        scanMaxProxAngleForwardLeft = wrapPI(z + SENSOR_PROXIMITY_FORWARD_LEFT_OFFSET);
      }
      if (scanMaxProxLeft < proxLeft && SCANNING_PROX_THRESHOLD < proxLeft)
      {
        scanMaxProxLeft = proxLeft;
        scanMaxProxAngleLeft = wrapPI(z + SENSOR_PROXIMITY_LEFT_OFFSET);
      }


      if (scanMinTOFRight > tofRight && SCANNING_TOF_THRESHOLD > tofRight && 0 < tofRight)
      {
        scanMinTOFRight = tofRight;
        scanMinTOFAngleRight = wrapPI(z);
      }
      if (scanMinTOFCenter > tofCenter && SCANNING_TOF_THRESHOLD > tofCenter && 0 < tofCenter)
      {
        scanMinTOFCenter = tofCenter;
        scanMinTOFAngleCenter = wrapPI(z);
      }
      if (scanMinTOFLeft > tofLeft && SCANNING_TOF_THRESHOLD > tofLeft && 0 < tofLeft)
      {
        scanMinTOFLeft = tofLeft;
        scanMinTOFAngleLeft = wrapPI(z);
      }

      zLast = z;

      // One revolution
      if (fabsf(z - zStart) < SCANNING_TURN_ERROR && millis() - turnOffsetTimestamp > SCANNING_TURN_OFFSET)
      {
        #ifdef SERIAL_ENABLE
        Serial.print("\tPROX:");
        Serial.print(" MAX R ");
        Serial.print(scanMaxProxRight);
        Serial.print(",");
        Serial.print(scanMaxProxAngleRight);
        Serial.print(" MAX FR ");
        Serial.print(scanMaxProxLeft);
        Serial.print(",");
        Serial.print(scanMaxProxAngleForwardRight);
        Serial.print(" MAX F ");
        Serial.print(scanMaxProxForward);
        Serial.print(",");
        Serial.print(scanMaxProxAngleForward);
        Serial.print(" MAX FL ");
        Serial.print(scanMaxProxForwardLeft);
        Serial.print(",");
        Serial.print(scanMaxProxAngleForwardLeft);
        Serial.print(" MAX L ");
        Serial.print(scanMaxProxLeft);
        Serial.print(",");
        Serial.print(scanMaxProxAngleLeft);
        Serial.print("\tTOF:");
        Serial.print(" MIN R ");
        Serial.print(scanMinTOFRight);
        Serial.print(",");
        Serial.print(scanMinTOFAngleRight);
        Serial.print(" MIN C ");
        Serial.print(scanMinTOFCenter);
        Serial.print(",");
        Serial.print(scanMinTOFAngleCenter);
        Serial.print(" MIN L ");
        Serial.print(scanMinTOFLeft);
        Serial.print(",");
        Serial.print(scanMinTOFAngleLeft);
        Serial.print("\t");
        #endif

        proxTargetAngle = 0;
        tofTargetAngle = 0;

        // Care for circularity of values (-pi,pi)
        if (fabsf(proxTargetAngle - scanMaxProxAngleForward) < 180 && scanMaxProxForward > 0)
        {
          proxTargetAngle += scanMaxProxAngleForward;
          proxCount++;
        }
        if (fabsf(proxTargetAngle - scanMaxProxAngleRight) < 180 && scanMaxProxRight > 0)
        {
          proxTargetAngle += scanMaxProxAngleRight;
          proxCount++;
        }
        if (fabsf(proxTargetAngle - scanMaxProxAngleForwardRight) < 180 && scanMaxProxForwardRight > 0)
        {
          proxTargetAngle += scanMaxProxAngleForwardRight;
          proxCount++;
        }
        if (fabsf(proxTargetAngle - scanMaxProxAngleForward) < 180 && scanMaxProxForwardLeft > 0)
        {
          proxTargetAngle += scanMaxProxAngleForward;
          proxCount++;
        }
        if (fabsf(proxTargetAngle - scanMaxProxAngleLeft) < 180 && scanMaxProxLeft > 0)
        {
          proxTargetAngle += scanMaxProxAngleLeft;
          proxCount++;
        }

        if (fabsf(tofTargetAngle - scanMinTOFAngleRight) < 180 && scanMinTOFRight < SCANNING_TOF_INF)
        {
          tofTargetAngle += scanMinTOFAngleRight;
          tofCount++;
        }
        if (fabsf(tofTargetAngle - scanMinTOFAngleCenter) < 180 && scanMinTOFCenter < SCANNING_TOF_INF)
        {
          tofTargetAngle += scanMinTOFAngleCenter;
          tofCount++;
        }
        if (fabsf(tofTargetAngle - scanMinTOFAngleLeft) < 180 && scanMinTOFLeft < SCANNING_TOF_INF)
        {
          tofTargetAngle += scanMinTOFAngleLeft;
          tofCount++;
        }

        Serial.println();
        Serial.print("prox: ");
        Serial.print(proxCount);
        Serial.print(", ");
        Serial.print(proxTargetAngle);
        Serial.print(" tof: ");
        Serial.print(tofCount);
        Serial.print(", ");
        Serial.print(tofTargetAngle);

        if (proxCount > 0)
        {
          proxTargetAngle /= proxCount;
        }
        if (tofCount > 0)
        {
          tofTargetAngle /= tofCount;
        }

        if (proxCount > 0 && (tofCount == 0 || fabsf(proxTargetAngle - tofTargetAngle) > 30))
        {
          #ifdef SERIAL_ENABLE
          Serial.print(" ==> bottle angle: ");
          Serial.print(proxTargetAngle);
          #endif

          bottleDetected = 1;
          targetAngle = proxTargetAngle;

          // Stop motors
          motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
          motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

          is_state = is_turn_stopping;
        }
        else if (tofCount > 0)
        {
          #ifdef SERIAL_ENABLE
          Serial.print(" ==> wall angle: ");
          Serial.print(tofTargetAngle);
          #endif

          tofTargetAngle = wrapPI(tofTargetAngle + 180);
          targetAngle = tofTargetAngle;

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

          // Stop motors
          motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
          motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

          flags[FLAG_NOTHING_DETECTED] = 1;

          is_state = is_off;
        }
      }
      else if (// TODO: Uncomment when sensors are well positioned
          // abs(proxDownLeft) > SCANNING_PROXIMITY_DOWN_THRESHOLD ||
          abs(proxDownRight) > SCANNING_PROXIMITY_DOWN_THRESHOLD)
      {
        // Stop motors
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

        is_state = is_back_off_start;
      }
    }
  }
  else if (is_state == is_back_off_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start back off");
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -SCANNING_BACK_OFF_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = -SCANNING_BACK_OFF_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    back_off_timestamp = millis();

    is_state = is_back_off;
  }
  else if (is_state == is_back_off)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("time: ");
    Serial.print(millis() - back_off_timestamp);
    #endif

    if (millis() - back_off_timestamp > SCANNING_BACK_OFF_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_back_off_stopping;
    }
  }
  else if (is_state == is_back_off_stopping)
  {
    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < SCANNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < SCANNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("back off stopped");
      #endif

      is_state = is_turn_start;
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
    float error = wrapPI(z - targetAngle);
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

      if (bottleDetected)
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > double-check bottle");
        #endif

        is_state = is_checking;
      }
      else
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > obstacle detected");
        #endif

        flags[FLAG_WALL_DETECTED] = 1;

        is_state = is_off;
      }
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

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateScanningExitRoutine(boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
