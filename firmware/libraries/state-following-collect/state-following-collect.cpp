/*
  state-following-collect.h - Following collect state methods
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "math.h"
#include "utils.h"
#include "config.h"
#include "sensors.h"
#include "actuators.h"

// Internal control flags
enum IState
{
  is_start,
  //is_calibrating,
  is_following_wall,
  is_stopping,
  is_forward,
  is_forward_stopping,
  is_waiting,
  is_backward,
  is_backward_stopping,
  is_turn_start,
  is_turning,
  is_turn_stopping,
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

static int proxDetectZero = 0;
static int proxForwardLeftZero = 0;
static int proxForwardZero = 0;
static int proxForwardRightZero = 0;

static long timestamp = 0;


void stateFollowingCollectEnterRoutine(boolean ledState[LED_COUNT],
                                       boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  is_state = is_start;
}

void stateFollowingCollectRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
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
      if (proxLookRight > proxLookLeft)
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

      // Open
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);


      // Set moving flag
      is_state = is_following_wall;
    }
  }

  else if (is_state == is_following_wall)
  {
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] > FOLLOWING_WALL_CORNER_DETECTED_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("slope detected");
      #endif

       // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;
    }
    else
    {
      int prox = getAverageProximityValue(proximityMeasurements, proximityToConsider) - proximityAmbientMeasurements[proximityToConsider];
      float error = (prox - FOLLOWING_WALL_DESIRED_WALL_DISTANCE);

      int tof = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);

      // Go fast when low error, else turn first
      float directionalSpeed = FOLLOWING_WALL_MAX_SPEED * (FOLLOWING_WALL_MAX_SPEED_ANGLE - fabsf(error)) / FOLLOWING_WALL_MAX_SPEED_ANGLE;
      if (directionalSpeed < FOLLOWING_WALL_MIN_SPEED)
      {
        directionalSpeed = FOLLOWING_WALL_MIN_SPEED;
      }
      if (directionalSpeed > FOLLOWING_WALL_MAX_SPEED)
      {
        directionalSpeed = FOLLOWING_WALL_MAX_SPEED;
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
  else if (is_state == is_stopping)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < FOLLOWING_WALL_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < FOLLOWING_WALL_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("stopped");
      #endif

      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = FOLLOWING_WALL_FORWARD_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = FOLLOWING_WALL_FORWARD_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      timestamp = millis();

      is_state = is_forward;
    }
  }
  else if (is_state == is_forward)
  {
    if (millis() - timestamp > FOLLOWING_WALL_FORWARD_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_forward_stopping;
    }
  }
  else if (is_state == is_forward_stopping)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < SLOPE_DOWN_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < SLOPE_DOWN_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("forward stopped");
      #endif

      proxDetectZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT];
      proxForwardLeftZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
      proxForwardZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
      proxForwardLeftZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];

      is_state = is_waiting;
    }
  }
  else if (is_state == is_waiting)
  {
    int proxDetect = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT] - proxDetect;

    int proxForwardLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT] - proxForwardLeft;
    int proxForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] - proxForward;
    int proxForwardRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT] - proxForwardRight;

    #ifdef SERIAL_ENABLE
    Serial.print("prox: ");
    Serial.print(proxDetect);
    Serial.print("\t");
    Serial.print(proxForwardLeft);
    Serial.print("\t");
    Serial.print(proxForward);
    Serial.print("\t");
    Serial.print(proxForwardRight);
    #endif

    if (proxDetect > FOLLOWING_WALL_DETECT_THRESHOLD)
    {
      // Close
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -FOLLOWING_WALL_BACKWARD_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -FOLLOWING_WALL_BACKWARD_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      timestamp = millis();

      is_state = is_backward;
    }
  }
  else if (is_state == is_backward)
  {
    if (millis() - timestamp > FOLLOWING_WALL_BACKWARD_DURATION)
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_backward_stopping;
    }
  }
  else if (is_state == is_backward_stopping)
  {
    if (motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT] < SLOPE_DOWN_STOPPING_THRESHOLD &&
        motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] < SLOPE_DOWN_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print("back stopped");
      #endif

      timestamp = millis();

      is_state = is_turn_start;
    }
  }
  else if (is_state == is_turn_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("desired angle: ");
    Serial.print(FOLLOWING_WALL_COLLECT_ANGLE);
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    is_state = is_turning;
  }
  else if (is_state == is_turning)
  {
    float error = wrapPI(estimatedAngle - FOLLOWING_WALL_COLLECT_ANGLE);
    float turningSpeed = error * FOLLOWING_WALL_COLLECT_TURNING_REACTIVITY;
    turningSpeed = max(turningSpeed, -FOLLOWING_WALL_COLLECT_TURNING_MAX_SPEED);
    turningSpeed = min(turningSpeed, FOLLOWING_WALL_COLLECT_TURNING_MAX_SPEED);
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

      is_state = is_turn_stopping;
    }
  }
  else if (is_state == is_turn_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < FOLLOWING_WALL_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < FOLLOWING_WALL_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" stopped");
      #endif

      flags[FLAG_SWALLOWED_BOTTLE] = 1;

      is_state = is_off;
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateFollowingCollectExitRoutine(boolean ledState[LED_COUNT],
                                    boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  // setServoAngle(ACTUATOR_SERVO_BAR_RIGHT_CLOSED, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  // setServoAngle(ACTUATOR_SERVO_BAR_LEFT_CLOSED, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
}
