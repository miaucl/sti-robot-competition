/*
sensor-proximity.h - The proximity sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "sensor-proximity.h"


SensorProximity::SensorProximity( unsigned int sensorId,
                                  unsigned int pin,
                                  unsigned int threshold = 0)
{
  // Sensor initialize message
  #ifdef SERIAL_ENABLE
  Serial.print("Init Sensor 'Proximity ");
  Serial.print(sensorId);
  Serial.println("' …");
  #endif

  // Save sensor id
  _sensorId = sensorId;
  // Save the pin
  _pin = pin;
  // Save the threshold
  threshold = threshold;
}

unsigned int SensorProximity::getValue()
{
  // Calculate the average of the past measurements
  unsigned int averageMeasurement = 0;
  for (unsigned int i = 0; i<MEASUREMENT_COUNT; i++)
  {
    averageMeasurement += _measurements[i];
  }
  averageMeasurement /= MEASUREMENT_COUNT;

  return averageMeasurement;
}


boolean SensorProximity::hasFlagRaised()
{
  unsigned int averageMeasurement = getValue();

  return (averageMeasurement > _ambient + _ambientVariance + threshold);
}


void SensorProximity::run(unsigned long now)
{
  // Save current execution timestamp
  _lastRunned = now;

  // Read the value from the analog sensor
  unsigned int measurement = analogRead(_pin);

  // Save the measurement
  _measurements[_measurementIndex++] = measurement;
  if (_measurementIndex >= MEASUREMENT_COUNT)
  {
    _measurementIndex = 0;
  }
}

void SensorProximity::exitState(State state)
{

}

void SensorProximity::enterState(State state)
{

}

void SensorProximity::calibrate()
{
  // Sensor calibration message
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'Proximity ");
  Serial.print(_sensorId);
  Serial.println("' …");
  #endif

  // Get a measure of the ambient light for the sensor
  _ambient = 0;
  _ambientVariance = 0;
  int _values[CALIBRATION_COUNT];
  for (unsigned int i = 0; i<CALIBRATION_COUNT; i++)
  {
    _values[i] = analogRead(_pin);
    _ambient += _values[i];
    delay(10); // Wait for 10 milliseconds between each measurement
  }
  _ambient /= CALIBRATION_COUNT;


  for (unsigned int i = 0; i<CALIBRATION_COUNT; i++)
  {
    _ambientVariance += abs(_values[i] - _ambient);
  }
  _ambientVariance /= CALIBRATION_COUNT;
  _ambientVariance *= 2;
}

unsigned int SensorProximity::getSensorId()
{
  return _sensorId;
}

unsigned long SensorProximity::getLastRunned()
{
  return _lastRunned;
}
