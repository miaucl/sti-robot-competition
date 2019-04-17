/*
  state-analysis.h - Analysis state methods
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
  is_following_wall,
  is_stopping,
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

void stateAnalysisEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Following Wall");
  #endif
}

void stateAnalysisRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                        int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                        float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                        double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                        boolean btnState[BTN_COUNT],
                        boolean ledState[LED_COUNT])
{
  Serial.print("following wall(");
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
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
  }

  // Start following wall
  if (is_state == is_start)
  {
    Serial.print("start\t");
    
    // determine if wall on the left or right is to follow
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT)>getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) || true)
    {
        followingWallRight = 1;
        Serial.println("Following wall on the right");
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
        followingWallLeft = 1;
        Serial.println("Following wall on the left");
        proximityToConsider = SENSOR_PROXIMITY_FORWARD_RIGHT;
        wallNearMotor = ACTUATOR_MOTOR_LEFT;
        wallNearMotorDirectionPin = ACTUATOR_MOTOR_LEFT_DIRECTION_PIN;
        wallNearMotorSpeedPin = ACTUATOR_MOTOR_LEFT_SPEED_PIN;
        
        wallFarMotor = ACTUATOR_MOTOR_RIGHT;
        wallFarMotorDirectionPin = ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN;
        wallFarMotorSpeedPin = ACTUATOR_MOTOR_RIGHT_SPEED_PIN;
    }
      
    // Set default calibrating speed
    motorSpeeds[ACTUATOR_MOTOR_RIGHT] = WALL_FOLLOWING_SPEED;
    motorSpeeds[ACTUATOR_MOTOR_LEFT] = WALL_FOLLOWING_SPEED;
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    // Set moving flag
    is_state = is_following_wall;
  }

  else if (is_state == is_following_wall)
  {
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD)-proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] > CORNER_DETECTED_THRESHOLD)
    {
       // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_stopping;   
      Serial.println("Corner detected");
    }
    else
    {
      float error = (getAverageProximityValue(proximityMeasurements, proximityToConsider) -
                    proximityAmbientMeasurements[proximityToConsider] +
                    DESIRED_WALL_DISTANCE); 
      Serial.print("Sensor value\t");
      Serial.print(getAverageProximityValue(proximityMeasurements, proximityToConsider));      
      Serial.print(" Error\t");
      Serial.print(error);
      // Adjust motor speed
      motorSpeeds[wallNearMotor] = WALL_FOLLOWING_SPEED + WALL_REACTIVITY * error;
      Serial.print(" Near\t");
      Serial.print(motorSpeeds[wallNearMotor]);
      motorSpeeds[wallFarMotor] = WALL_FOLLOWING_SPEED - WALL_REACTIVITY * error;
      Serial.print(" Far\t");
      Serial.println(motorSpeeds[wallFarMotor]);    
      writeMotorSpeed(motorSpeeds, wallNearMotor, wallNearMotorDirectionPin, wallNearMotorSpeedPin);
      writeMotorSpeed(motorSpeeds, wallFarMotor, wallFarMotorDirectionPin, wallFarMotorSpeedPin);
   
    }

  }
    
  else if (is_state == is_stopping)
  {
    Serial.print("motor: ");
    Serial.println(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    if (motorPositionMeasurements[ACTUATOR_MOTOR_LEFT] < WALL_FOLLOWING_STOPPING_THRESHOLD &&
        motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT] < WALL_FOLLOWING_STOPPING_THRESHOLD)
    {
        
        
    //exit -> raise flag
    }
  }    
    
}


void stateAnalysisExitRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
