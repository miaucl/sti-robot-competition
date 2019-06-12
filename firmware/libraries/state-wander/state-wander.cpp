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
  is_back_off_start,
  is_back_off,
  is_back_off_stopping,
  is_off
};
static IState is_state = is_start;
static long turn_timestamp = 0;
static long back_off_timestamp = 0;

static int proxDownLeftZero = 0;
static int proxDownRightZero = 0;

static float pitchZero = 0;
static float rollZero = 0;

static boolean ignoreRight = false;
static boolean ignoreLeft = false;

static boolean backOffLeft = false;

static long offsetTimestamp = 0;
//static long openTimestamp = 0;


void stateWanderEnterRoutine( boolean ledState[LED_COUNT],
                              boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  proxDownLeftZero = 0;
  proxDownRightZero = 0;
  pitchZero = 0;
  rollZero = 0;

  is_state = is_start;
}

void stateWanderRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                        int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                        float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                        int servoAngles[ACTUATOR_SERVO_COUNT],
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
  int proximityDownLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT] - proxDownLeftZero;
  int proximityDownRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT] - proxDownRightZero;
  int proximityDetect = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT];
  float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
  float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
  float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);

  float pitch = getMedianIMUPitchOrientationValue(imuMeasurements) - pitchZero;
  float roll = getMedianIMURollOrientationValue(imuMeasurements) - rollZero;


  // Detect accitentally swallowed bottles
  if (proximityDetect > WANDER_DETECT_BOTTLE_THRESHOLD)
  {
    flags[FLAG_SWALLOWED_BOTTLE] = 1;

    is_state = is_off;
  }



  // Start moving
  if (is_state == is_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start\t");
    #endif

    // // Close
    // servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
    // servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;
    // writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
    // writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
    //
    // openTimestamp = millis();

    // Set default wandering speed
    stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = flags[FLAG_ON_PLATFORM] ? WANDER_PLATFORM_SPEED : WANDER_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = flags[FLAG_ON_PLATFORM] ? WANDER_PLATFORM_SPEED : WANDER_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    // Get current zero level
    proxDownLeftZero = proximityDownLeft;
    proxDownRightZero = proximityDownRight;

    // Get current pitch and roll zero level
    pitchZero = pitch;
    rollZero = roll;


    // timestamp for offset
    offsetTimestamp = millis();

    if (flags[FLAG_ON_PLATFORM])
    {
      ignoreLeft = 1;
      ignoreRight = 1;
    }

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
    Serial.print(proximityLeft);
    Serial.print(", ");
    Serial.print(proximityForwardLeft);
    Serial.print(", ");
    Serial.print(proximityForward);
    Serial.print(", ");
    Serial.print(proximityForwardRight);
    Serial.print(", ");
    Serial.print(proximityRight);
    Serial.print(", ");
    Serial.print(proximityDownLeft);
    Serial.print(", ");
    Serial.print(proximityDownRight);
    Serial.print(", ");
    Serial.print(proximityDetect);
    Serial.print(" pitch: ");
    Serial.print(pitch);
    Serial.print(" roll: ");
    Serial.print(roll);
    #endif

    if (ignoreRight)
    {
      if ((proximityRight > WANDER_PROXIMITY_SIDE_THRESHOLD && proximityRight < WANDER_PROXIMITY_HIGH_THRESHOLD) ||
      (proximityForwardRight > WANDER_PROXIMITY_SIDE_THRESHOLD && proximityForwardRight < WANDER_PROXIMITY_HIGH_THRESHOLD))
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
      if ((proximityLeft > WANDER_PROXIMITY_SIDE_THRESHOLD && proximityLeft < WANDER_PROXIMITY_HIGH_THRESHOLD) ||
      (proximityForwardLeft > WANDER_PROXIMITY_SIDE_THRESHOLD && proximityForwardLeft < WANDER_PROXIMITY_HIGH_THRESHOLD))
      {
        proximityLeft = 0;
        proximityForwardLeft = 0;
      }
      else
      {
        ignoreLeft = false;
      }
    }

    if (abs(roll) > WANDER_ROLL_THRESHOLD || abs(pitch) > WANDER_PITCH_THRESHOLD)
    {
      Serial.print(abs(pitch));
      Serial.print(abs(roll));
      Serial.print(abs(roll) > WANDER_ROLL_THRESHOLD || abs(pitch) > WANDER_PITCH_THRESHOLD);
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_back_off_start;
    }
    else if ((tofLeft < WANDER_TOF_LEFT__MOVING_THRESHOLD && tofLeft > 0) ||
             (tofCenter < WANDER_TOF_CENTER__MOVING_THRESHOLD && tofCenter > 0) ||
             (tofRight < WANDER_TOF_RIGHT__MOVING_THRESHOLD && tofRight > 0))
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;
    }
    else if (millis() - offsetTimestamp < WANDER_OFFSET)
    {
    }
    else if (proximityRight > WANDER_PROXIMITY_SIDE_THRESHOLD ||
        proximityForwardRight > WANDER_PROXIMITY_THRESHOLD ||
        proximityForward > WANDER_PROXIMITY_THRESHOLD ||
        proximityForwardLeft > WANDER_PROXIMITY_THRESHOLD ||
        proximityLeft > WANDER_PROXIMITY_SIDE_THRESHOLD ||
        (-proximityDownLeft > WANDER_PROXIMITY_DOWN_THRESHOLD && flags[FLAG_ON_PLATFORM]) ||
        (-proximityDownRight > WANDER_PROXIMITY_DOWN_THRESHOLD && flags[FLAG_ON_PLATFORM]))
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
      #ifdef SERIAL_ENABLE
      Serial.print("tof: ");
      Serial.print(tofLeft);
      Serial.print(", ");
      Serial.print(tofCenter);
      Serial.print(", ");
      Serial.print(tofRight);
      Serial.print(" ir: ");
      Serial.print(proximityLeft);
      Serial.print(", ");
      Serial.print(proximityForwardLeft);
      Serial.print(", ");
      Serial.print(proximityForward);
      Serial.print(", ");
      Serial.print(proximityForwardRight);
      Serial.print(", ");
      Serial.print(proximityRight);
      Serial.print(", ");
      Serial.print(proximityDownLeft);
      Serial.print(", ");
      Serial.print(proximityDownRight);
      #endif

      if ((tofLeft > WANDER_TOF_LEFT_THRESHOLD || tofLeft == 0) &&
          (tofCenter > WANDER_TOF_CENTER_THRESHOLD || tofCenter == 0) &&
          (tofRight > WANDER_TOF_RIGHT_THRESHOLD || tofRight == 0) &&
          (-proximityDownLeft < WANDER_PROXIMITY_DOWN_THRESHOLD || !flags[FLAG_ON_PLATFORM]) &&
          (-proximityDownRight < WANDER_PROXIMITY_DOWN_THRESHOLD || !flags[FLAG_ON_PLATFORM]))
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" horizontal prox detection");
        #endif

        flags[FLAG_ELEMENT_DETECTED] = 1;

        is_state = is_off;
      }
      else if ((-proximityDownLeft > WANDER_PROXIMITY_DOWN_THRESHOLD && flags[FLAG_ON_PLATFORM]) ||
               (-proximityDownRight > WANDER_PROXIMITY_DOWN_THRESHOLD && flags[FLAG_ON_PLATFORM]))
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" vertical prox detection");
        #endif

        backOffLeft = -proximityDownLeft > -proximityDownRight;

        is_state = is_back_off_start;

        stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
      }
      else
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" tof detection");
        #endif
        is_state = is_turn_start;
      }
    }
  }
  else if (is_state == is_back_off_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start back off");
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -WANDER_PLATFORM_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = -WANDER_PLATFORM_SPEED;
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

    if (millis() - back_off_timestamp > WANDER_BACK_OFF_DURATION)
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
    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < WANDER_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < WANDER_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("back off stopped");
      #endif

      is_state = is_turn_start;
    }
  }
  else if (is_state == is_turn_start)
  {
    boolean turnLeft;
    if (tofRight > 0) turnLeft = false;
    else if (tofLeft > 0) turnLeft = true;
    else if (tofCenter > 0) turnLeft = millis() % 2 == 0;
    else if (backOffLeft) turnLeft = false;
    else turnLeft = true;

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

    if (millis() - turn_timestamp > WANDER_TURN_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      // // Close
      // servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      // servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      // writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      // writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

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

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateWanderExitRoutine(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
