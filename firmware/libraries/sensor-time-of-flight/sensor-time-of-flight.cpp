/*
sensor-time-of-flight.h - The time-of-flight sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "sensor-time-of-flight.h"



SensorTimeOfFlight::SensorTimeOfFlight( unsigned int sensorId,
                                        unsigned int triggerPin,
                                        unsigned int echoPin,
                                        unsigned int threshold = 0)
{
  // Sensor initialize message
  #ifdef SERIAL_ENABLE
  Serial.print("Init Sensor 'TimeOfFlight ");
  Serial.print(sensorId);
  Serial.println("' …");
  #endif

  // Save sensor id
  _sensorId = sensorId;
  // Save the pin
  _triggerPin = triggerPin;
  _echoPin = echoPin;
  // Save the threshold
  threshold = threshold;

  // Configure pings
  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT);

}

unsigned int SensorTimeOfFlight::getValue()
{
  return _measurements[0];
  // Calculate the average of the past measurements
  // unsigned int averageMeasurement = 0;
  // for (unsigned int i = 0; i<MEASUREMENT_COUNT; i++)
  // {
  //   averageMeasurement += _measurements[i];
  // }
  // averageMeasurement /= MEASUREMENT_COUNT;
  //
  // return averageMeasurement;
}


boolean SensorTimeOfFlight::hasFlagRaised()
{
  // unsigned int averageMeasurement = getValue();
  //
  // return (averageMeasurement > _ambient + _ambientVariance + threshold);
}


void SensorTimeOfFlight::run(unsigned long now)
{
  // Save current execution timestamp
  _lastRunned = now;

  // Check if echo is received
  if (!_waitingForEcho)
  {
    // Echo has been received
  }
  else
  {
    // No echo receieved
    _waitingForEcho = false;


  }


  // Clears the trigger
  digitalWrite(_triggerPin, LOW);
  delayMicroseconds(2);

  // Sets the trigger on HIGH state for 10 micro seconds
  digitalWrite(_triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_triggerPin, LOW);
  // Save the timestamp for the trigger
  _triggerMillis = millis();
  // Raise flag that a signal has been sent
  _waitingForEcho = true;

  Serial.println(millis());
  digitalWrite(10, LOW);

  // Read the value from the analog sensor
  // unsigned int measurement = analogRead(_pin);
  //
  // // Save the measurement
  // _measurements[_measurementIndex++] = measurement;
  // if (_measurementIndex >= MEASUREMENT_COUNT)
  // {
  //   _measurementIndex = 0;
  // }
}

void SensorTimeOfFlight::loop()
{
  //Serial.println(millis() - _triggerMillis);
  if (_waitingForEcho && _triggerMillis + 1 < millis() && digitalRead(_echoPin) == LOW)
  {
    _waitingForEcho = false;
    _echoMillis = millis();

    _measurements[0] = _echoMillis - _triggerMillis;

    // Serial.println(_measurements[0]);

    // analogWrite(10, map(_measurements[0], 0, 50, 0, 255));
  }
}

void SensorTimeOfFlight::exitState(State state)
{

}

void SensorTimeOfFlight::enterState(State state)
{

}

void SensorTimeOfFlight::calibrate()
{
  // Sensor calibration message
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'TimeOfFlight ");
  Serial.print(_sensorId);
  Serial.println("' …");
  #endif

  pinMode(10, OUTPUT);
}

unsigned int SensorTimeOfFlight::getSensorId()
{
  return _sensorId;
}

unsigned long SensorTimeOfFlight::getLastRunned()
{
  return _lastRunned;
}
