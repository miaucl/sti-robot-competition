/*
  state-analysis.h - Following state methods
  Created by Mirko Indumi, 2019.
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
  is_checking,
    
  is_swallowing_starting,
  is_swallowing,
  is_swallowing_stopping,
    
  is_stopping,
  is_off
};
static IState is_state = is_start;
static int maxProx = 0;
static int maxProxVal = 0;
static int beforeProxVal = 0;
static long swallowingTimestamp = 0;
static int proxDetect = 0;
static int proxDetectLeft = 0;
static int proxDetectRight = 0;
static int bottleInRobot = 0;


void stateSwallowingEnterRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = HIGH;

  #ifdef SERIAL_ENABLE
  Serial.println("Swallowing Bottle");
  #endif
}

void stateSwallowingRoutine(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                          int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                          int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                          int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
                          double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                          double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT],
                          boolean btnState[BTN_COUNT],
                          boolean ledState[LED_COUNT])
{
  Serial.print("Swallowing bottle(");
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

  // Start swallowing bottle
  if (is_state == is_start)
  {
    Serial.print("start\t");

      
    maxProxVal =  getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT]; 
      
    // determine where the bottle is
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT]>maxProxVal)
    {
        maxProx=1;
        maxProxVal =  getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT]; 
    }
    else if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD)- proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]>maxProxVal)
    {
        maxProx=2;
        maxProxVal =  getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD)- proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]; 
    }
    else if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT]>maxProxVal)
    {
        maxProx=3;
        maxProxVal =  getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT]; 
    }
    else if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT]>maxProxVal)
    {
        maxProx=4;
        maxProxVal =  getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT)- proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT]; 
    }
    
    Serial.print("maxProxVal: ");
    Serial.print(maxProxVal);
    Serial.print("\t maxProx");
    Serial.println(maxProx);  
      
      
    is_state = is_turn_start;
      
    if (maxProx == 2) //bottle is already in front of robot
    {
        is_state = is_checking;
        Serial.println(" Bottle is already in front of robot");
    }
  }
    
  else if (is_state == is_turn_start)
  {
    Serial.print(" turn\t");
    // Set default turning speed
    if (maxProx<2)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = SWALLOWING_TURNING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = -SWALLOWING_TURNING_SPEED;
    }
    else
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = -SWALLOWING_TURNING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = SWALLOWING_TURNING_SPEED;
    }
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
    writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

    
    is_state = is_turning;
  }  
  else if (is_state == is_turning)
  {
    Serial.print("Front IR: ");
    Serial.println(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]);
    
    // turn until measurements of front proximities get inferior to the maximal measurement of before
    if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD] > maxProxVal-SWALLOWING_TURNING_THRESHOLD)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_turn_stopping;
      Serial.println("Now the bottle is in front of robot");
    }
  }
  else if (is_state == is_turn_stopping)
  {

    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < SWALLOWING_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < SWALLOWING_STOPPING_THRESHOLD)
    {
      is_state = is_checking;
    }
  }
  else if (is_state == is_checking)
  {
      int proximityForwardRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT];
      int proximityForward = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD];
      int proximityForwardLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT];
      float tofRight = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
      float tofCenter = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
      float tofLeft = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);

      if (!(tofCenter < SWALLOWING_TOF_THRESHOLD && tofCenter > 0) && proximityForward > SWALLOWING_PROXIMITY_THRESHOLD)
      {
        Serial.println("Woooohooooo, it is a bottle");
        is_state = is_swallowing_starting;
      }
      else 
      {
        Serial.plrint("TOF Value: Center")
        Serial.print(tofCenter);
        Serial.println(" NOOOOO, OBSTACLE");
        
        //Exit to wander state
      }
  }
    
  else if (is_state == is_swallowing_starting)
  {
      writeServoAngle[ACTUATOR_SERVO_BAR_RIGHT_OPEN,ACTUATOR_SERVO_BAR_RIGHT,ACTUATOR_SERVO_BAR_RIGHT_PIN];
      writeServoAngle[ACTUATOR_SERVO_BAR_LEFT_OPEN,ACTUATOR_SERVO_BAR_LEFT,ACTUATOR_SERVO_BAR_LEFT_PIN];
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = SWALLOWING_SPEED;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = SWALLOWING_SPEED;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
      swallowingTimestamp = millis();
      
      proxDetect = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_LEFT];
      
      if (getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_RIGHT] > proxDetect)
      {
          proxDetect = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_RIGHT];
      }

      is_state = is_swallowing;      
  }     
    
  else if (is_state == is_swallowing)
  {
    Serial.print("time: ");
    Serial.println(millis() - swallowingTimestamp);
    
      
    proxDetectRight = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_RIGHT];
    proxDetectLeft = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT_LEFT];
      
    if (millis() - swallowingTimestamp > SWALLOWING_DURATION )
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      is_state = is_swallowing_stopping;
      Serial.println("NOOOOO, Could not get bottle!");
     
      
    }
    else if (proxDetectRight > proxDetect + SWALLOWING_BOTTLE_DETECTION_THRESHOLD || proxDetectLeft > proxDetect + SWALLOWING_BOTTLE_DETECTION_THRESHOLD)
    {
      // Stop motors
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
      
      bottleInRobot = 1;
      is_state = is_swallowing_stopping;
      Serial.println("I got you little bastard, hehehehe!");
    }
  }
 
  else if (is_state == is_swallowing_stopping)
  {
    Serial.print("motor: ");
    Serial.print(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]);
    Serial.print("\t");
    if (fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_LEFT]) < SWALLOWING_STOPPING_THRESHOLD &&
        fabsf(motorPositionMeasurements[ACTUATOR_MOTOR_RIGHT]) < SWALLOWING_STOPPING_THRESHOLD)
    {
      writeServoAngle[ACTUATOR_SERVO_BAR_RIGHT_CLOSED,ACTUATOR_SERVO_BAR_RIGHT,ACTUATOR_SERVO_BAR_RIGHT_PIN];
      writeServoAngle[ACTUATOR_SERVO_BAR_LEFT_CLOSED,ACTUATOR_SERVO_BAR_LEFT,ACTUATOR_SERVO_BAR_LEFT_PIN];
      if(bottleInRobot == 1)
      {
          //Exit to return state
      }
      else
      {
          //Exit to wander state
      }
    }
  }
    
    
    


}


void stateSwallowingExitRoutine(boolean ledState[LED_COUNT])
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
}
