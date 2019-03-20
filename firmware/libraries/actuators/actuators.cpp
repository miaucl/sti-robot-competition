/*
  leds.cpp - Led functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"

#include <Servo.h>
#include <AutoPID.h>

static AutoPID *autoPID[ACTUATOR_MOTOR_COUNT];
static double targetMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static double controlMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static double measuredMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static long ticks[ACTUATOR_MOTOR_COUNT] = {0};
static long lastMotorUpdateTimestamp[ACTUATOR_MOTOR_COUNT] = {micros(), micros()};

void encoderTickRight() { ticks[ACTUATOR_MOTOR_RIGHT]++; }
void encoderTickLeft() { ticks[ACTUATOR_MOTOR_LEFT]++; }


void configureMotor(int id,
                    int directionPin,
                    int speedPin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Configure Actuator 'Motor ");
  Serial.print(id);
  Serial.print("' on pins '");
  Serial.print(directionPin);
  Serial.print("' and '");
  Serial.print(speedPin);
  Serial.println("'");
  #endif

  pinMode(directionPin, OUTPUT);
  pinMode(speedPin, OUTPUT);

  // GLOBAL INT CONFIG
  if (id == ACTUATOR_MOTOR_RIGHT) attachInterrupt(digitalPinToInterrupt(ACTUATOR_MOTOR_RIGHT_INT_PIN), encoderTickRight, CHANGE);
  if (id == ACTUATOR_MOTOR_LEFT) attachInterrupt(digitalPinToInterrupt(ACTUATOR_MOTOR_LEFT_INT_PIN), encoderTickLeft, CHANGE);


  // Initialize the PID
  autoPID[id] = new AutoPID(&measuredMotorSpeeds[id], &targetMotorSpeeds[id], &controlMotorSpeeds[id], 0.f, 255.f, ACTUATOR_MOTOR_PID_KP, ACTUATOR_MOTOR_PID_KI, ACTUATOR_MOTOR_PID_KD);
  autoPID[id]->setTimeStep(10);
  autoPID[id]->setBangBang(1, 1);
  autoPID[id]->reset();
}

void writeMotorSpeed( boolean motorDirections[ACTUATOR_MOTOR_COUNT],
                      double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                      int id,
                      int directionPin,
                      int speedPin)
{
  // Save value
  targetMotorSpeeds[id] = motorSpeeds[id];

  digitalWrite(directionPin, motorDirections[id]);
}


void stopMotor( int id,
                int directionPin,
                int speedPin)
{
  // Set to 0
  targetMotorSpeeds[id] = 0;

  digitalWrite(speedPin, 0);
  digitalWrite(directionPin, LOW);

  // Reset the PID
  autoPID[id]->reset();
}

void updateMotorSpeedControl( int id,
                              int directionPin,
                              int speedPin)
{
  long motorUpdateTimestamp = micros();

  // Micros overflow protection
  if (motorUpdateTimestamp > lastMotorUpdateTimestamp[id])
  {
    // Disable to interrupts
    //noInterrupts();

    // Calculate from ticks in interval to meter / seconds
    int ticksCount = ticks[id];
    ticks[id] = 0;
    long ticksPerSecond = ticksCount * 1000000 / (motorUpdateTimestamp - lastMotorUpdateTimestamp[id]);
    float radPerSecond = ((float)ticksPerSecond) / ACTUATOR_MOTOR_ENCODER_RESOLUTION / ACTUATOR_MOTOR_TRANSMISSION * 2 * M_PI;
    float meterPerSecond = radPerSecond * ACTUATOR_MOTOR_DIAMETER;

    measuredMotorSpeeds[id] = meterPerSecond;
  }

  // Run PID Controller
  autoPID[id]->run();

  // Serial.print(targetMotorSpeeds[id]);
  // Serial.print(" ");
  // Serial.print(measuredMotorSpeeds[id]);
  // Serial.print(" ");
  // Serial.print(controlMotorSpeeds[id]);
  // Serial.println("");

  // Stop if value is 0
  if (targetMotorSpeeds[id] == 0)
  {
    autoPID[id]->reset();
    analogWrite(speedPin, 0);
  }
  // PID control value
  else
  {
    analogWrite(speedPin, controlMotorSpeeds[id]);
  }

  // Save timestamp
  lastMotorUpdateTimestamp[id] = motorUpdateTimestamp;
}



Servo servos[2];

/**
 * Configure the servo
 */
void configureServo(int id,
                    int pin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Configure Actuator 'Servo ");
  Serial.print(id);
  Serial.print("' on pin '");
  Serial.print(pin);
  Serial.println("'");
  #endif

  servos[id].attach(pin);
}

/**
 * Write the servo angle
 */
void writeServoAngle( int servoAngles[ACTUATOR_SERVO_COUNT],
                      int id,
                      int pin)
{
  servos[id].write(servoAngles[id]);
}
