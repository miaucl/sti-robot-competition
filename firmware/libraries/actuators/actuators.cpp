/*
  leds.cpp - Led functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"
#include "math.h"

#include <Servo.h>
#include <AutoPID.h>

#define MOTOR_SPEED_STEP (ACTUATOR_MOTOR_MAX_SLEW_RATE * PERIOD / 1000.f) // PERIOD IN MILLIS

static AutoPID *autoPID[ACTUATOR_MOTOR_COUNT];
static double targetMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static double intermediateMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static double controlMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static double measuredMotorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
static long ticks[ACTUATOR_MOTOR_COUNT] = {0};
static long lastMotorUpdateTimestamp[ACTUATOR_MOTOR_COUNT] = {micros(), micros()};

static boolean rawInput = false;

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
  autoPID[id] = new AutoPID(&measuredMotorSpeeds[id], &intermediateMotorSpeeds[id], &controlMotorSpeeds[id], -255.f, 255.f, ACTUATOR_MOTOR_PID_KP, ACTUATOR_MOTOR_PID_KI, ACTUATOR_MOTOR_PID_KD);
  autoPID[id]->setTimeStep(10);
  autoPID[id]->setBangBang(1, 1);
  autoPID[id]->reset();
}

float calculateNextIntermediateSpeed(float targetMotorSpeed, float intermediateMotorSpeed)
{
  if (intermediateMotorSpeed == targetMotorSpeed || rawInput) return targetMotorSpeed;
  // Serial.print(targetMotorSpeed);
  // Serial.print("\t");
  // Serial.print(intermediateMotorSpeed);
  // Serial.print("\t");
  float step = MOTOR_SPEED_STEP; // PERIOD IN MILLISECONDS
  if (targetMotorSpeed < intermediateMotorSpeed) step *= -1; // Go Down
  float newIntermediateMotorSpeed = intermediateMotorSpeed + step;
  //Serial.println(newIntermediateMotorSpeed);
  if (abs(newIntermediateMotorSpeed - targetMotorSpeed) < 2 * MOTOR_SPEED_STEP) return targetMotorSpeed;
  else return newIntermediateMotorSpeed;
}

void writeMotorSpeed( double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                      int id,
                      int directionPin,
                      int speedPin)
{
  // Save value
  targetMotorSpeeds[id] = motorSpeeds[id];
  intermediateMotorSpeeds[id] = measuredMotorSpeeds[id];

  // Bound speed
  targetMotorSpeeds[id] = min(max(targetMotorSpeeds[id], -ACTUATOR_MOTOR_SPEED_MAX), ACTUATOR_MOTOR_SPEED_MAX);

  rawInput = false;
  intermediateMotorSpeeds[id] = calculateNextIntermediateSpeed(targetMotorSpeeds[id], measuredMotorSpeeds[id]);
}

void writeRawMotorSpeed(double motorSpeeds[ACTUATOR_MOTOR_COUNT],
                        int id,
                        int directionPin,
                        int speedPin)
{
  // Save value
  targetMotorSpeeds[id] = motorSpeeds[id];
  intermediateMotorSpeeds[id] = measuredMotorSpeeds[id];

  // Bound speed
  targetMotorSpeeds[id] = min(max(targetMotorSpeeds[id], -ACTUATOR_MOTOR_SPEED_MAX), ACTUATOR_MOTOR_SPEED_MAX);

  rawInput = true;
  intermediateMotorSpeeds[id] = targetMotorSpeeds[id];
}


void stopMotor( int id,
                int directionPin,
                int speedPin)
{
  // Set to 0
  targetMotorSpeeds[id] = 0;
  intermediateMotorSpeeds[id] = 0;

  digitalWrite(speedPin, 0);
  digitalWrite(directionPin, LOW);

  // Reset the PID
  autoPID[id]->reset();
}

void updateMotorSpeedControl( int id,
                              int directionPin,
                              int speedPin,
                              double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT])
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

    // Go Backwards
    if (controlMotorSpeeds[id] < 0) meterPerSecond *= -1;

    // Export measurement of speed
    motorSpeedMeasurements[id] = meterPerSecond;

    measuredMotorSpeeds[id] = meterPerSecond;
  }

  // Get new intermediate motor speed
  intermediateMotorSpeeds[id] = calculateNextIntermediateSpeed(targetMotorSpeeds[id], intermediateMotorSpeeds[id]);

  // Run PID Controller
  autoPID[id]->run();

  // Test outputs CSV
  // if (id == 0)
  // {
  //   Serial.print(intermediateMotorSpeeds[0]);
  //   Serial.print(',');
  //   Serial.print(measuredMotorSpeeds[0]);
  //   Serial.print(',');
  //   Serial.println(controlMotorSpeeds[0]);
  // }

  // Stop if value is 0
  if (intermediateMotorSpeeds[id] == 0)
  {
    autoPID[id]->reset();
    analogWrite(speedPin, 0);
  }
  // PID control value
  else
  {
    analogWrite(speedPin, abs(controlMotorSpeeds[id]));
    digitalWrite(directionPin, controlMotorSpeeds[id] < 0);
  }

  // Save timestamp
  lastMotorUpdateTimestamp[id] = motorUpdateTimestamp;
}


#define SERVO_ANGLE_STEP (1000.f / ACTUATOR_SERVO_STEPS_PER_SECOND) // PERIOD IN MILLIS

Servo servos[2];
static int targetServoAngle[ACTUATOR_SERVO_COUNT] = {0};
static int intermediateServoAngle[ACTUATOR_SERVO_COUNT] = {0};
static long lastServoUpdateTimestamp[ACTUATOR_MOTOR_COUNT] = {millis(), millis()};


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

void resetServoAngle( int servoAngles[ACTUATOR_SERVO_COUNT],
                      int id,
                      int pin)
{
  servos[id].write(servoAngles[id]);
  targetServoAngle[id] = servoAngles[id];
  intermediateServoAngle[id] = servoAngles[id];
}

void writeServoAngle( int servoAngles[ACTUATOR_SERVO_COUNT],
                      int id,
                      int pin)
{
  targetServoAngle[id] = servoAngles[id];

}

void updateServoAngleControl( int id,
                              int pin)
{
  long servoUpdateTimestamp = millis();

  if ((servoUpdateTimestamp - lastServoUpdateTimestamp[id]) > SERVO_ANGLE_STEP)
  {
    if (targetServoAngle[id] > intermediateServoAngle[id]) intermediateServoAngle[id]++;
    if (targetServoAngle[id] < intermediateServoAngle[id]) intermediateServoAngle[id]--;

    lastServoUpdateTimestamp[id] = servoUpdateTimestamp;

    servos[id].write(intermediateServoAngle[id]);
  }
}
