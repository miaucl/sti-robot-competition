/*
  controller.h - The controller of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "controller.h"


Controller::Controller(SensorProximity *sensorProximityList, SensorTimeOfFlight *sensorTimeOfFlightList)
{
  // Controller initialize message
  #ifdef SERIAL_ENABLE
  Serial.println("Init Controller …");
  #endif

  // Save sensors
  _sensorProximityList = sensorProximityList;
  _sensorTimeOfFlightList = sensorTimeOfFlightList;
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
  // Serial.print("State:\t");
  // switch (getState())
  // {
  //   case s_initialize: Serial.print("INIT"); break;
  //   case s_run: Serial.print("RUN");
  // }
  // Serial.print("\t");
  //
  // // Proximity thresholds
  // Serial.print("Prox THs:\t");
  // if (_sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_OFFSET + SENSOR_PROXIMITY_RIGHT].hasFlagRaised()) Serial.print("R ");
  // if (_sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_OFFSET + SENSOR_PROXIMITY_FORWARD_RIGHT].hasFlagRaised()) Serial.print("FR ");
  // if (_sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_OFFSET + SENSOR_PROXIMITY_FORWARD].hasFlagRaised()) Serial.print("F ");
  // if (_sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_OFFSET + SENSOR_PROXIMITY_FORWARD_LEFT].hasFlagRaised()) Serial.print("FL ");
  // if (_sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_OFFSET + SENSOR_PROXIMITY_LEFT].hasFlagRaised()) Serial.print("L ");
  // if (_sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_OFFSET + SENSOR_PROXIMITY_BACKWARD].hasFlagRaised()) Serial.print("B ");

  // Time-of-flight thresholds
  // Serial.print("TOF THs:\t");
  // Serial.println(_sensorTimeOfFlightList[SENSOR_TIME_OF_FLIGHT_RIGHT].getValue());


  // Serial.println("");


  #endif
}

void Controller::run(unsigned long now)
{
  // Save current execution timestamp
  _lastRunned = now;

  // Log values
	logging();
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

unsigned long Controller::getLastRunned()
{
  return _lastRunned;
}
