/*
  controller.h - The controller of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Controller_h
#define Controller_h

#include "Arduino.h"
#include <Thread.h>
#include <ThreadController.h>
#include "config.h"
#include "sensors.h"


/**
 * The controller class inheriting Thread
 */
class Controller: public Thread
{
  public:

    /**
     * Constructor
     */
    Controller(Sensors *sensors, void (*callback)(void) = NULL, unsigned long _interval = 0);

    /**
     * Start the robot
     */
    void start();

    /**
     * Get the current state
     */
    State getState();

  private:

    /**
     * Internal variable for the current state
     */
    State _state = s_initialize;

    /**
     * Sensors of the robot
     */
    Sensors *_sensors;

    /**
     * Logging function used to log repeatedly values
     */
    void logging();

    /**
     * The overrode run function of the Thread class, being called at a fix
     * frequency rate.
     */
    void run() override;

    /**
     * Make a state transition from current _state to newState
     */
    void setState(State newState);
};

#endif
