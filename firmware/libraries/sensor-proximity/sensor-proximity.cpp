/*
sensor-proximity.h - The proximity sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "sensor-proximity.h"


SensorProximity::SensorProximity( unsigned int sensorId,
                                  unsigned int pin,
                                  unsigned int threshold = 0,
                                  void (*callback)(void),
                                  unsigned long _interval) : Thread(callback, _interval)
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
  // Save the theshold
  theshold = theshold;


  // Set the frequency
  setInterval(SENSOR_PROXIMITY_FREQUENCY);
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

  return (averageMeasurement > _ambient + _ambientVariance + FLAG_THRESHOLD);
}


void SensorProximity::run()
{
  // Read the value from the analog sensor
  unsigned int measurement = analogRead(_pin);

  // Save the measurement
  _measurements[_measurementIndex++] = measurement;
  if (_measurementIndex >= MEASUREMENT_COUNT)
  {
    _measurementIndex = 0;
  }

  // Release thread
  runned();
}

SensorProximity::exitState(State state)
{

}

SensorProximity::enterState(State state)
{

}

SensorProximity::calibrate()
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
  Serial.print("Ambient: ");
  Serial.println(_ambient);


  for (unsigned int i = 0; i<CALIBRATION_COUNT; i++)
  {
    _ambientVariance += abs(_values[i] - _ambient);
  }
  _ambientVariance /= CALIBRATION_COUNT;
  _ambientVariance *= 2;

  Serial.print("Ambient Variance: ");
  Serial.println(_ambientVariance);
}

SensorProximity::getSensorId()
{
  return _sensorId;
}
