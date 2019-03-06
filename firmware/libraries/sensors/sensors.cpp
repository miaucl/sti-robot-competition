/*
  sensors.h - The sensors of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "sensors.h"
#include "sensor.h"
#include "sensor-proximity.h"


Sensors::Sensors( SensorProximity *sensorProximityList,
                  unsigned int sensorProximityCount,
                  void (*callback)(void),
                  unsigned long _interval) : Thread(callback, _interval)
{
  // Sensors initialize message
  #ifdef SERIAL_ENABLE
  Serial.println("Init Sensors …");
  #endif

  // Save the proximity sensor list and add to the thread list
  _sensorProximityList = sensorProximityList;
  _sensorProximityCount = sensorProximityCount;
  #ifdef SERIAL_ENABLE
  Serial.print(_sensorProximityCount);
  Serial.println(" Proximity Sensors registered …");
  #endif

  // Set the frequency
  setInterval(SENSORS_FREQUENCY);
}

Sensors::exitState(State state)
{

}

Sensors::enterState(State state)
{

}

unsigned int Sensors::getProximityValue(unsigned int sensorId)
{
  SensorProximity *s = _getProximitySensor(sensorId);
  if (s == nullptr)
  {
    return 0;
  }
  else
  {
    return s->getValue();
  }
}

unsigned int Sensors::hasProximityFlagRaised(unsigned int sensorId)
{
  SensorProximity *s = _getProximitySensor(sensorId);
  if (s == nullptr)
  {
    return 0;
  }
  else
  {
    return s->hasFlagRaised();
  }
}

SensorProximity* Sensors::_getProximitySensor(unsigned int sensorId)
{
  for (unsigned int i = 0; i<_sensorProximityCount; i++)
  {
    if (_sensorProximityList[i].getSensorId() == sensorId)
    {
      return &_sensorProximityList[i];
    }
  }
  return nullptr;
}

void Sensors::run()
{
  // Release thread
  runned();
}

void Sensors::calibrate()
{
  // Sensors calibration message
  #ifdef SERIAL_ENABLE
  Serial.println("Calibrate Sensors …");
  #endif

  // Calibrate proximity sensors
  for (unsigned int i = 0; i<_sensorProximityCount; i++)
  {
    _sensorProximityList[i].calibrate();
  }
}
