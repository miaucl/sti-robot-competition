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
void writeMotorSpeed( boolean motorDirections[ACTUATOR_MOTOR_COUNT],
                      double motorSpeeds[ACTUATOR_MOTOR_COUNT],
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
                              int speedPin);

/**
 * Configure the servo
 */
void configureServo(int id,
                    int pin);

/**
 * Write the servo angle
 */
void writeServoAngle( int servoAngles[ACTUATOR_SERVO_COUNT],
                      int id,
                      int pin);


#endif
