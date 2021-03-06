/*
  actuators.h - Actuators functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Actuator_h
#define Actuator_h

/**
 * Configure the motor
 */
void configureMotor(int id,
                    int directionPin,
                    int speedPin);

/**
 * Write the motor speeds
 */
void writeMotorSpeed( double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                      int id,
                      int directionPin,
                      int speedPin);

/**
 * Write the raw motor speeds
 */
void writeRawMotorSpeed(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        int id,
                        int directionPin,
                        int speedPin);


/**
 * Stop the motor
 */
void stopMotor( int id,
                int directionPin,
                int speedPin);

/**
 * Update the motor control
 */
void updateMotorSpeedControl( int id,
                              int directionPin,
                              int speedPin,
                              double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT]);

/**
 * Configure the servo
 */
void configureServo(int id,
                    int pin);

/**
 * Reset the servo angle
 */
void setServoAngle( int servoAngles[ACTUATOR_SERVO_COUNT],
                      int id,
                      int pin);

/**
 * Write the servo angle
 */
void writeServoAngle( int servoAngles[ACTUATOR_SERVO_COUNT],
                      int id,
                      int pin);

/**
 * Update the servo control
 */
void updateServoAngleControl( int id,
                              int pin);


#endif
