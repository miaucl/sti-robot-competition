/*
  state-following-slope-drop.h - Following slope and drop state methods
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "math.h"
#include "utils.h"
#include "config.h"
#include "sensors.h"
#include "actuators.h"

#define USE_RIGHT_WALL 0

// Internal control flags
enum IState
{
  is_start,
  //is_calibrating,
  is_orienting,
  is_following_wall,
  is_drop_turn_start,
  is_drop_turning,
  is_drop_turn_stopping,
  is_drop,
  is_wiggle,
  is_off
};
static IState is_state = is_start;
static int followingWallRight = 0;
static int followingWallLeft = 0;
static int proximityToConsider = 0;
static int wallNearMotor = 0;
static int wallNearMotorDirectionPin = 0;
static int wallNearMotorSpeedPin = 0;
static int wallFarMotor = 0;
static int wallFarMotorDirectionPin = 0;
static int wallFarMotorSpeedPin = 0;

static long dropTimestamp = 0;
static long wiggleTimestamp = 0;

static float proxDownLeftZero = 0;
static float proxDownRightZero = 0;
static float proxForwardZero = 0;

static float pitchZero = 0;
static float rollZero = 0;


void stateFollowingSlopeDropEnterRoutine( boolean ledState[LED_COUNT],
                                          boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  is_state = is_start;
}

void stateFollowingSlopeDropRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                                    int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                                    int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                                    int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                                    float estimatedAngle,
                                    float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                                    double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                                    double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                                    int servoAngles[ACTUATOR_SERVO_COUNT],
                                    boolean btnState[BTN_COUNT],
                                    boolean ledState[LED_COUNT],
                                    boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("following(");
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

  // Start following wall
  if (is_state == is_start)
  {
    int proxLookLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT];
    int proxLookRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT];

    if (proxLookLeft > 0 || proxLookRight > 0)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("start:\t");
      Serial.print(proxLookLeft);
      Serial.print(" / ");
      Serial.print(proxLookRight);
      #endif

      // determine if wall on the left or right is to follow
      //if (proxLookRight > proxLookLeft)
      if (USE_RIGHT_WALL)
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > wall: right");
        #endif

        flags[FLAG_FOLLOWING_RIGHT_SIDE] = 1;

        followingWallRight = 1;
        proximityToConsider = SENSOR_PROXIMITY_FORWARD_RIGHT;
        wallNearMotor = ACTUATOR_MOTOR_RIGHT;
        wallNearMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
        wallNearMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;

        wallFarMotor = ACTUATOR_MOTOR_LEFT;
        wallFarMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
        wallFarMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;
      }
      else
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" > wall: left");
        #endif

        flags[FLAG_FOLLOWING_RIGHT_SIDE] = 0;

        followingWallLeft = 1;
        proximityToConsider = SENSOR_PROXIMITY_FORWARD_LEFT;
        wallNearMotor = ACTUATOR_MOTOR_LEFT;
        wallNearMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
        wallNearMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;

        wallFarMotor = ACTUATOR_MOTOR_RIGHT;
        wallFarMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
        wallFarMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;
      }

      // Set following speed to 0
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      pitchZero = getMedianIMUPitchOrientationValue(imuMeasurements);
      rollZero = getMedianIMURollOrientationValue(imuMeasurements);

      // Set moving flag
      is_state = is_orienting;
    }
  }
  else if (is_state == is_orienting)
  {
    float error = wrapPI(estimatedAngle - FOLLOWING_DROP_ANGLE);
    float turningSpeed = error * FOLLOWING_DROP_REACTIVITY;
    turningSpeed = max(turningSpeed, -FOLLOWING_DROP_MAX_SPEED);
    turningSpeed = min(turningSpeed, FOLLOWING_DROP_MAX_SPEED);
    #ifdef SERIAL_ENABLE
    Serial.print("orienting: ");
    Serial.print(error);
    Serial.print(", ");
    Serial.print(turningSpeed);
    #endif

    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -turningSpeed;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = turningSpeed;
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    if (fabsf(error) < FOLLOWING_DROP_TURNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > stopping");
      #endif
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_following_wall;
    }
  }
  else if (is_state == is_following_wall)
  {
    float proxDownLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT] - proxDownLeftZero;
    float proxDownRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT] - proxDownRightZero;
    float proxForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] - proxForwardZero;

    #ifdef SERIAL_ENABLE
    Serial.print("\tprox d: ");
    Serial.print(proxDownLeft);
    Serial.print(", ");
    Serial.print(proxDownRight);
    Serial.print("\t forward: ");
    Serial.print(proxDownRight);
    #endif

    if (-proxDownLeft > SLOPE_DOWN_PROXIMITY_DOWN_THRESHOLD ||
        -proxDownRight > SLOPE_DOWN_PROXIMITY_DOWN_THRESHOLD ||
        proxForward < -SLOPE_DOWN_PROXIMITY_FORWARD_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("slope detected");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = FOLLOWING_DROP_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = FOLLOWING_DROP_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_drop_turn_start;
    }
    else
    {
      int prox = getAverageProximityValue(proximityMeasurements, proximityToConsider) - proximityAmbientMeasurements[proximityToConsider];
      float error = (prox - FOLLOWING_DROP_DESIRED_WALL_DISTANCE);

      int tof = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);

      // Go fast when low error, else turn first
      float directionalSpeed = FOLLOWING_DROP_MAX_SPEED * (FOLLOWING_WALL_MAX_SPEED_ANGLE - fabsf(error)) / FOLLOWING_WALL_MAX_SPEED_ANGLE;
      if (directionalSpeed < FOLLOWING_DROP_MIN_SPEED)
      {
        directionalSpeed = FOLLOWING_DROP_MIN_SPEED;
      }
      if (directionalSpeed > FOLLOWING_DROP_MAX_SPEED)
      {
        directionalSpeed = FOLLOWING_DROP_MAX_SPEED;
      }

      // Slow down when aproaching something
      if (tof > 0)
      {
        directionalSpeed *= FOLLOWING_WALL_APPROACHING_FACTOR;
      }

      // 0 value passing controller for angle
      float rotationalSpeed = FOLLOWING_WALL_REACTIVITY * error;

      // Adjust motor speed
      motorSpeeds[wallNearMotor] = directionalSpeed + rotationalSpeed;
      motorSpeeds[wallFarMotor] = directionalSpeed - rotationalSpeed;

      #ifdef SERIAL_ENABLE

      Serial.print("\tprox: ");
      Serial.print(prox);
      Serial.print("\terror: ");
      Serial.print(error);
      Serial.print("\ttof: ");
      Serial.print(tof);
      Serial.print("\tspeed: ");
      Serial.print(motorSpeeds[wallNearMotor]);
      Serial.print(", ");
      Serial.print(motorSpeeds[wallFarMotor]);
      #endif

      writeRawMotorSpeed(motorSpeeds, wallNearMotor, wallNearMotorDirectionPin, wallNearMotorSpeedPin);
      writeRawMotorSpeed(motorSpeeds, wallFarMotor, wallFarMotorDirectionPin, wallFarMotorSpeedPin);
    }
  }
  else if (is_state == is_drop_turn_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("desired angle: ");
    Serial.print(FOLLOWING_DROP_ANGLE);
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


    is_state = is_drop_turning;

  }
  else if (is_state == is_drop_turning)
  {
    float error = wrapPI(estimatedAngle - FOLLOWING_DROP_ANGLE);
    float turningSpeed = error * FOLLOWING_DROP_MIN_SPEED;
    turningSpeed = max(turningSpeed, -FOLLOWING_DROP_MIN_SPEED);
    turningSpeed = min(turningSpeed, FOLLOWING_DROP_MIN_SPEED);
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

    if (fabsf(error) < FOLLOWING_WALL_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > stopping");
      #endif
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_drop_turn_stopping;
    }
  }
  else if (is_state == is_drop_turn_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < FOLLOWING_WALL_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < FOLLOWING_WALL_STOPPING_THRESHOLD)
    {
      float error = wrapPI(estimatedAngle - FOLLOWING_DROP_ANGLE);
      if (fabsf(error) >= FOLLOWING_DROP_TURNING_STOPPING_THRESHOLD)
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" not good enough: ");
        Serial.print(error);
        #endif

        is_state = is_drop_turn_start;
      }
      else
      {
        #ifdef SERIAL_ENABLE
        Serial.print(" stopped");
        #endif

        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = FOLLOWING_DROP_SPEED;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = FOLLOWING_DROP_SPEED;
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

        servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_RAMP;
        servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_RAMP;
        writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
        writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);


        dropTimestamp = millis();

        is_state = is_drop;
      }
    }
  }
  else if (is_state == is_drop)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("dropping");
    #endif

    if (millis() - dropTimestamp > FOLLOWING_DROP_DURATION)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > timeout");
      #endif

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = FOLLOWING_DROP_WIGGLE_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -FOLLOWING_DROP_WIGGLE_SPEED;
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      wiggleTimestamp = millis();
      is_state = is_wiggle;
    }
  }
  else if (is_state == is_wiggle)
  {
    float pitch = getMedianIMUPitchOrientationValue(imuMeasurements) - pitchZero;
    float roll = getMedianIMURollOrientationValue(imuMeasurements) - rollZero;

    #ifdef SERIAL_ENABLE
    Serial.print("wiggle: ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(roll);
    #endif

    if (abs(pitch) < FOLLOWING_DROP_PITCH_THRESHOLD && abs(roll) < FOLLOWING_DROP_ROLL_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > flat again");
      #endif

      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);


      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      flags[FLAG_ON_PLATFORM] = 0;

      is_state = is_off;
    }
    else if (millis() - wiggleTimestamp > FOLLOWING_DROP_WIGGLE_DURATION)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > switch");
      #endif

      wiggleTimestamp = millis();

      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -motorSpeeds[ACTUATOR_MOTOR_RIGHT];
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -motorSpeeds[ACTUATOR_MOTOR_LEFT];
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateFollowingSlopeDropExitRoutine(boolean ledState[LED_COUNT],
                                        boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  // setServoAngle(ACTUATOR_SERVO_BAR_RIGHT_CLOSED, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  // setServoAngle(ACTUATOR_SERVO_BAR_LEFT_CLOSED, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
}
