/*
sensor-proximity.h - The proximity sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "sensor-proximity.h"


SensorProximity::SensorProximity( unsigned int sensorId,
                                  unsigned int pin,
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


  // Set the frequency
  setInterval(SENSOR_PROXIMITY_FREQUENCY);
}


void SensorProximity::run()
{
  Serial.print(analogRead(_pin));

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
}

SensorProximity::getSensorId()
{
  return _sensorId;
}
