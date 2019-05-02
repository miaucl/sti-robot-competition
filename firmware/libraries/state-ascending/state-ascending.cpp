/*
  state-analysis.h - Ascending state methods
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
  //is_calibrating,
  is_turn_start_180,
  is_turning_180,
  is_turn_stopping_180,
  is_following_wall,
  is_detecting_corner,
  is_turn_start_90,
  is_turning_90,
  is_turn_stopping_90,
  is_ascending_slope,
  is_stopping,
  is_off
};
static IState is_state = is_start;
static float zStart = 0;
static int wallCount = 0;
static int detectingThreshold = 0;
static int desiredZ=0;
static long slopeTimestamp = 0;

static int proximityToConsider = 0;
static int wallNearMotor = 0;
static int wallNearMotorDirectionPin = 0;
static int wallNearMotorSpeedPin = 0;
static int wallFarMotor = 0;
static int wallFarMotorDirectionPin = 0;
static int wallFarMotorSpeedPin = 0;

void stateAscendingEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Ascending to platform");
  #endif
}

void stateAscendingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                          int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                          int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                          int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                          int servoAngles[ACTUATOR_SERVO_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT])
{
  Serial.print("Ascending to Platform(");
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

  // Start following wall
  if (is_state == is_start)
  {
    Serial.print("start\t");
    is_state = is_turn_start_180; //this state is called after dropping a bottle, the robot is oriented in direction of the corner -> turn 180 degrees, then the robot follows the wall until it is on the platform
      
  }

  else if (is_turn_start_180)
  {
    Serial.print("Start turning(");
    Serial.print(is_state);
    Serial.println(")\t");
    proximityToConsider = SENSOR_PROXIMITY_FORWARD_RIGHT;
    wallNearMotor = ACTUATOR_MOTOR_RIGHT;
    wallNearMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
    wallNearMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;

    wallFarMotor = ACTUATOR_MOTOR_LEFT;
    wallFarMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
    wallFarMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;  
      
          // Set default turning speed (always turn right/left)
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -ASCENDING_TURNING_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = ASCENDING_TURNING_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


    zStart = getMedianIMUZOrientationValue(imuMeasurements);
    is_state = is_turning_180;
    int desiredZ = zStart+180;
    if (desiredZ > 180) desiredZ -= 360;
  }
  else if (is_state == is_turning_180)   
  {
      
     float z = getMedianIMUZOrientationValue(imuMeasurements);
     Serial.print("Z = ");
     Serial.println(z);
     float error = fabsf(desiredZ - z); 
     if (error < ASCENDING_TURNING_THRESHOLD)
     {
         motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
         motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
         writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
         writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
         
         wallCount = 1;
         is_state = is_turn_stopping_180;
     }    
  }
  else if (is_state == is_turn_stopping_180)
  {

    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < ASCENDING_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < ASCENDING_STOPPING_THRESHOLD)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = ASCENDING_WALL_FOLLOWING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = ASCENDING_WALL_FOLLOWING_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
      is_state = is_following_wall;
    }
  }
    
  else if (is_state == is_following_wall)
  { //when having followed just 1 wall we want to detect the corner, else the slope
    Serial.print("Is following wall(");
    Serial.print(is_state);
    Serial.print(")\t");
    if (wallCount==1) detectingThreshold = ASCENDING_CORNER_DETECTED_THRESHOLD;
    else detectingThreshold = ASCENDING_SLOPE_DETECTED_THRESHOLD;

    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD)-proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] > detectingThreshold)
    {
      if (wallCount==1) 
      {
       // Stop motors
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);


        is_state = is_turn_start_90;
        Serial.print("Corner Detected: ");
        Serial.println(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD)-proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]);          
      }
      else 
      {
        is_state = is_ascending_slope;
        slopeTimestamp = millis();
        Serial.print("Corner Detected: ");
        Serial.println(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD)-proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]);       
      }

    }
    else
    {
      float error = (getAverageProximityValue(proximityMeasurements, proximityToConsider) -
                    proximityAmbientMeasurements[proximityToConsider] -
                    ASCENDING_DESIRED_WALL_DISTANCE);
      Serial.print("Sensor value\t");
      Serial.print(getAverageProximityValue(proximityMeasurements, proximityToConsider));
      Serial.print(" Error\t");
      Serial.print(error);
      // Adjust motor speed
      motorSpeeds[wallNearMotor] = ASCENDING_WALL_FOLLOWING_SPEED + ASCENDING_WALL_REACTIVITY * error;
      Serial.print(" Near\t");
      Serial.print(motorSpeeds[wallNearMotor]);
      motorSpeeds[wallFarMotor] = ASCENDING_WALL_FOLLOWING_SPEED - ASCENDING_WALL_REACTIVITY * error;
      Serial.print(" Far\t");
      Serial.println(motorSpeeds[wallFarMotor]);
      writeRawMotorSpeed(motorSpeeds, wallNearMotor, wallNearMotorDirectionPin, wallNearMotorSpeedPin);
      writeRawMotorSpeed(motorSpeeds, wallFarMotor, wallFarMotorDirectionPin, wallFarMotorSpeedPin);
    }

  }
  else if (is_state == is_turn_start_90)
  {
      Serial.print("Turning 90 degrees(");
      Serial.print(is_state);
      Serial.println(")\t");
      if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < ASCENDING_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < ASCENDING_STOPPING_THRESHOLD)
    {
          // Set default turning speed (always turn left)
        motorSpeeds[ACTUATOR_MOTOR_RIGHT] = ASCENDING_TURNING_SPEED;
        motorSpeeds[ACTUATOR_MOTOR_LEFT] = -ASCENDING_TURNING_SPEED;
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
        writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
            
        zStart = getMedianIMUZOrientationValue(imuMeasurements);
        is_state = is_turning_180;
        int desiredZ = zStart+90.f;
        if (desiredZ > 180.f) desiredZ -= 360;
          
        is_state = is_turning_90;
    }
  }
  else if (is_state == is_turning_90)   
  {
      
     float z = getMedianIMUZOrientationValue(imuMeasurements);
     Serial.print("Z = ");
     Serial.println(z);
     float error = fabsf(desiredZ - z); 
     if (error < ASCENDING_TURNING_THRESHOLD)
     {
         motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
         motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
         writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
         writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
         
         wallCount = 2;
         is_state = is_turn_stopping_90;
     }    
  }
  else if (is_state == is_turn_stopping_90)
  {

    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < ASCENDING_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < ASCENDING_STOPPING_THRESHOLD)
    {
      Serial.print("Opening the barrier in order not to crash while ascending slope(");
      Serial.print(is_state);
      Serial.println(")\t");
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
        
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = ASCENDING_WALL_FOLLOWING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = ASCENDING_WALL_FOLLOWING_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
      is_state = is_following_wall;
    }
  }
  
  else if (is_state == is_ascending_slope)
  {
      Serial.print("Ascending slope(");
      Serial.print(is_state);
      Serial.println(")\t");
      int proxLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT)-proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
      int proxRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT)-proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];      
      int errorSlope = proxLeft - proxRight;
      
      if (millis() - slopeTimestamp > ASCENDING_SLOPE_TRANISTION_DURATION)
      {
            motorSpeeds[ACTUATOR_MOTOR_RIGHT] = ASCENDING_WALL_FOLLOWING_SPEED + ASCENDING_SLOPE_REACTIVITY*errorSlope;
            motorSpeeds[ACTUATOR_MOTOR_LEFT] = ASCENDING_WALL_FOLLOWING_SPEED - ASCENDING_SLOPE_REACTIVITY*errorSlope;
      }
      if (millis() - slopeTimestamp > ASCENDING_SLOPE_TRANISTION_DURATION && max(proxLeft,proxRight)<ASCENDING_SLOPE_PROX_THRESHOLD)
      {
            motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
            motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
            writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
            writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
            is_state == is_stopping;
      } 
  }
    
  else if (is_state == is_stopping)
  {
    Serial.print("Reached Platform(");
    Serial.print(is_state);
    Serial.print(")\t");
    Serial.print("motor: ");
    Serial.println(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    if (motorPositionMeasurements[ACTUATOR_MOTOR_LEFT] < ASCENDING_STOPPING_THRESHOLD &&
        motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT] < ASCENDING_STOPPING_THRESHOLD)
    {
        servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
        servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
        writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
        writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

    //exit -> raise flag
    }
  }

}


void stateAscendingExitRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
