/*
  state-wander.h - Wander state methods
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

// Internal control flags
enum IState
{
  is_start,
  is_turn_start,
  is_turning,
  is_turn_stopping,
  is_braitenberg,
  is_back_off_start,
  is_back_off,
  is_back_go_close,
  is_back_go_close_stopping,
  is_back_turn_start,
  is_back_turning,
  is_back_turn_stopping,
  is_off
};

static IState is_state = is_start;
static float zStart = 0;
static float zDesired = 0;
static float z = 0;
static float error = 0;
static int proxDownLeftZero = 0;
static int proxDownRightZero = 0;

static long braitenbergTimestamp = 0;
static long back_off_timestamp = 0;

static int directionalSpeedCount = 0;

void stateReturningEnterRoutine(boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  directionalSpeedCount = 0;
  proxDownLeftZero = 0;
  proxDownRightZero = 0;

  is_state = is_start;
}

void stateReturningRoutine( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                            int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                            int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                            int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                            float estimatedAngle,
                            double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                            double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT],
                            boolean btnState[BTN_COUNT],
                            boolean ledState[LED_COUNT],
                            boolean flags[FLAG_COUNT])
{
  #ifdef SERIAL_ENABLE
  Serial.print("returning(");
  Serial.print(is_state);
  Serial.print(")\t");
  Serial.print(estimatedAngle);
  Serial.print("\t");
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
    #ifdef SERIAL_ENABLE
    Serial.print("desired angle: ");
    Serial.print((flags[FLAG_ON_PLATFORM]) ? RETURNING_TARGET_ANGLE_PLATFORM : RETURNING_TARGET_ANGLE);
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    proxDownLeftZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT];
    proxDownRightZero = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT];


    is_state = is_turning;

  }
  else if (is_state == is_turning)
  {
    float error = wrapPI(estimatedAngle - ((flags[FLAG_ON_PLATFORM]) ? RETURNING_TARGET_ANGLE_PLATFORM : RETURNING_TARGET_ANGLE));
    float turningSpeed = error * RETURNING_TURNING_REACTIVITY;
    turningSpeed = max(turningSpeed, -RETURNING_TURNING_MAX_SPEED);
    turningSpeed = min(turningSpeed, RETURNING_TURNING_MAX_SPEED);
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

    if (fabsf(error) < RETURNING_STOPPING_THRESHOLD)
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

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < RETURNING_TURNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < RETURNING_TURNING_STOPPING_THRESHOLD)
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
    // // Down sensors
    // int proximityDownLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT] - proxDownLeftZero;
    // int proximityDownRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT] - proxDownRightZero;
    //
    // if ((abs(proximityDownLeft) > WANDER_PROXIMITY_DOWN_THRESHOLD ||
    //      abs(proximityDownRight) > WANDER_PROXIMITY_DOWN_THRESHOLD) && flags[FLAG_ON_PLATFORM])
    // {
    //   #ifdef SERIAL_ENABLE
    //   Serial.print(" > down detection");
    //   #endif
    //
    //   is_state = is_back_off_start;
    //
    //   // Stop motors
    //   motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
    //   motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    //   writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    //   writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    // }
    // else
    // {
      float rightSpeed = 0;
      float leftSpeed = 0;

      // Bias
      rightSpeed += RETURNING_BRAITENBERG_BIAS;
      leftSpeed += RETURNING_BRAITENBERG_BIAS;

      // Right tof sensor
      float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
      if (tofRight > 0)
      {
        rightSpeed += RETURNING_BRAITENBERG_TOF_RIGHT_RIGHT_FACTOR * (RETURNING_BRAITENBERG_TOF_MAX - tofRight);
        leftSpeed += RETURNING_BRAITENBERG_TOF_RIGHT_LEFT_FACTOR * (RETURNING_BRAITENBERG_TOF_MAX - tofRight);
      }

      // Forward tof sensor
      float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
      if (tofCenter > 0)
      {
        rightSpeed += RETURNING_BRAITENBERG_TOF_FORWARD_FACTOR * (RETURNING_BRAITENBERG_TOF_MAX - tofCenter);
        leftSpeed += RETURNING_BRAITENBERG_TOF_FORWARD_FACTOR * (RETURNING_BRAITENBERG_TOF_MAX - tofCenter);
      }

      // Left tof sensor
      float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);
      if (tofLeft > 0)
      {
        rightSpeed += RETURNING_BRAITENBERG_TOF_LEFT_RIGHT_FACTOR * (RETURNING_BRAITENBERG_TOF_MAX - tofLeft);
        leftSpeed += RETURNING_BRAITENBERG_TOF_LEFT_LEFT_FACTOR * (RETURNING_BRAITENBERG_TOF_MAX - tofLeft);
      }


      // Forward prox sensor (only if tof sees something)
      if (tofCenter > 0)
      {
        int proxForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
        rightSpeed += RETURNING_BRAITENBERG_PROX_FORWARD_FACTOR * proxForward;
        leftSpeed += RETURNING_BRAITENBERG_PROX_FORWARD_FACTOR * proxForward;        
      }

      // Forward prox left sensor
      int proxLeftForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
      rightSpeed += RETURNING_BRAITENBERG_PROX_FORWARD_LEFT_RIGHT_FACTOR * abs(proxLeftForward);
      leftSpeed += RETURNING_BRAITENBERG_PROX_FORWARD_LEFT_LEFT_FACTOR * abs(proxLeftForward);

      // Forward prox right sensor
      int proxRightForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
      rightSpeed += RETURNING_BRAITENBERG_PROX_FORWARD_RIGHT_RIGHT_FACTOR * abs(proxRightForward);
      leftSpeed += RETURNING_BRAITENBERG_PROX_FORWARD_RIGHT_LEFT_FACTOR * abs(proxRightForward);


      // Prox left sensor
      int proxLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT];
      rightSpeed += RETURNING_BRAITENBERG_PROX_LEFT_RIGHT_FACTOR * abs(proxLeft);
      leftSpeed += RETURNING_BRAITENBERG_PROX_LEFT_LEFT_FACTOR * abs(proxLeft);

      // Prox right sensor
      int proxRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT];
      rightSpeed += RETURNING_BRAITENBERG_PROX_RIGHT_RIGHT_FACTOR * abs(proxRight);
      leftSpeed += RETURNING_BRAITENBERG_PROX_RIGHT_LEFT_FACTOR * abs(proxRight);


      // Angle
      float angleError = wrapPI(estimatedAngle - ((flags[FLAG_ON_PLATFORM]) ? RETURNING_TARGET_ANGLE_PLATFORM : RETURNING_TARGET_ANGLE));
      rightSpeed -= RETURNING_BRAITENBERG_ANGLE_ERROR_FACTOR * angleError;
      leftSpeed += RETURNING_BRAITENBERG_ANGLE_ERROR_FACTOR * angleError;

      rightSpeed = max(rightSpeed, -RETURNING_BRAITENBERG_MAX_SPEED);
      rightSpeed = min(rightSpeed, RETURNING_BRAITENBERG_MAX_SPEED);
      leftSpeed = max(leftSpeed, -RETURNING_BRAITENBERG_MAX_SPEED);
      leftSpeed = min(leftSpeed, RETURNING_BRAITENBERG_MAX_SPEED);
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

      // No directional speed
      if (millis() - braitenbergTimestamp > RETURNING_BRAITENBERG_THRESHOLD &&
          fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < RETURNING_BRAITENBERG_STOPPING_THRESHOLD &&
          fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < RETURNING_BRAITENBERG_STOPPING_THRESHOLD)
      {
        directionalSpeedCount++;

        if (directionalSpeedCount > RETURNING_BRAITENBERG_STOPPING_COUNT)
        {
          #ifdef SERIAL_ENABLE
          Serial.print(" > stopping");
          #endif
          motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
          motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
          writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

          is_state = is_back_go_close;
        }
      }
      else
      {
        directionalSpeedCount = 0;
      }
    // }
  }
  else if (is_state == is_back_off_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("start back off");
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -RETURNING_BACK_OFF_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = -RETURNING_BACK_OFF_SPEED;
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

    if (millis() - back_off_timestamp > RETURNING_BACK_OFF_DURATION)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_braitenberg;
    }
  }
  else if (is_state == is_back_go_close)
  {
    // Forward prox sensor
    int proxForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];

    #ifdef SERIAL_ENABLE
    Serial.print("go close: ");
    Serial.print(proxForward);
    #endif

    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = RETURNING_GO_CLOSE_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = RETURNING_GO_CLOSE_SPEED;
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    if (proxForward > RETURNING_GO_CLOSE_FRONT_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > close enough");
      #endif
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_back_go_close_stopping;
    }
  }
  else if (is_state == is_back_go_close_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < RETURNING_TURNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < RETURNING_TURNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" stopped");
      #endif

      is_state = is_back_turn_start;
    }
  }
  else if (is_state == is_back_turn_start)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("desired angle: ");
    Serial.print(((flags[FLAG_ON_PLATFORM]) ? RETURNING_TARGET_ANGLE_PLATFORM_DEPART : RETURNING_TARGET_ANGLE));
    #endif
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


    is_state = is_back_turning;

  }
  else if (is_state == is_back_turning)
  {
    float error = wrapPI(estimatedAngle - ((flags[FLAG_ON_PLATFORM]) ? RETURNING_TARGET_ANGLE_PLATFORM_DEPART : RETURNING_TARGET_ANGLE));
    float turningSpeed = error * RETURNING_TURNING_REACTIVITY;
    turningSpeed = max(turningSpeed, -RETURNING_TURNING_MAX_SPEED);
    turningSpeed = min(turningSpeed, RETURNING_TURNING_MAX_SPEED);
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

    if (fabsf(error) < RETURNING_TURNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" > stopping");
      #endif
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_back_turn_stopping;
    }
  }
  else if (is_state == is_back_turn_stopping)
  {

    if (fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) < RETURNING_TURNING_STOPPING_THRESHOLD &&
        fabsf(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]) < RETURNING_TURNING_STOPPING_THRESHOLD)
    {
      #ifdef SERIAL_ENABLE
      Serial.print(" stopped");
      #endif

      flags[FLAG_RETURNED] = 1;

      is_state = is_off;
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println();
  #endif
}


void stateReturningExitRoutine( boolean ledState[LED_COUNT],
                                boolean flags[FLAG_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
