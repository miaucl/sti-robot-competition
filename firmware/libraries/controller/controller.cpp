/*
  controller.h - The controller of the robot
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "controller.h"


Controller::Controller(void (*callback)(void), unsigned long _interval) : Thread(callback, _interval)
{
  // Controller initialize message
  #ifdef SERIAL_ENABLE
  Serial.println("Init Controller …");
  #endif

  // Set the frequency
  setInterval(CONTROLLER_FREQUENCY);
}

void Controller::start()
{
  // Set the state to 's_run' for starting
  setState(s_run);
}

void Controller::logging()
{
  #ifdef SERIAL_ENABLE
  Serial.print("State:\t");
  Serial.println(_state);
  #endif
}

void Controller::run()
{
  // Log values
	logging();

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
