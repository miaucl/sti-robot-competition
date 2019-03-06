/*
  controller.h - The controller of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "controller.h"


Controller::Controller(Sensors *sensors, void (*callback)(void), unsigned long _interval) : Thread(callback, _interval)
{
  // Controller initialize message
  #ifdef SERIAL_ENABLE
  Serial.println("Init Controller …");
  #endif

  // Save the sensors
  _sensors = sensors;

  // Set the frequency
  setInterval(CONTROLLER_FREQUENCY);
}

void Controller::start()
{
  // Set the state to 's_run' for starting
  setState(s_run);
}

State Controller::getState()
{
  return _state;
}

void Controller::logging()
{
  #ifdef SERIAL_ENABLE
  // State
  Serial.print("State:\t");
  switch (getState())
  {
    case s_initialize: Serial.print("INITIALIZE"); break;
    case s_run: Serial.print("RUN");
  }
  Serial.print("\t");

  // Proximity thresholds
  Serial.print("Prox THs:\t");
  if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_RIGHT)) Serial.print("R ");
  if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_FORWARD_RIGHT)) Serial.print("FR ");
  if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_FORWARD)) Serial.print("F ");
  if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_FORWARD_LEFT)) Serial.print("FL ");
  if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_LEFT)) Serial.print("L ");
  if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_BACKWARD)) Serial.print("B ");


  Serial.println("");


  #endif
}

void Controller::run()
{
  // Log values
	logging();

  // Serial.println(_sensors->getProximityValue(SENSOR_PROXIMITY_RIGHT));
  // if (_sensors->hasProximityFlagRaised(SENSOR_PROXIMITY_RIGHT))
  // {
  //   Serial.println("FLAG");
  // }

  // Release thread
  runned();
}

void Controller::setState(State newState)
{
  #ifdef SERIAL_ENABLE
  Serial.print("State Transition '");
  Serial.print(_state);
  Serial.print("' > '");
  Serial.print(newState);
  Serial.println("'");
  #endif

  _state = newState;
}
