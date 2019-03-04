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

/**
 * The possible states of the robot can have
 */
enum State
{
  s_initialize,       // Initialization state, only active during set up
  s_run,
  s_turn,
  s_wait,
  s_panic             // Panic state, when something unexpected happened
};

/**
 * Every class depending on state must be inherited by this class
 * exitState and enterState are getting called during a state transition
 */
class Module
{
  public:
    /**
     * Function being called when the module should prepare to exit a state
     * Abstract class
     */
    virtual exitState(State exitState) = 0;
    /**
     * Function being called when the module should enter a new state
     * Abstract class
     */
    virtual enterState(State enterState) = 0;
  private:
    /**
     * Internal variable for the current state
     */
    State _state = s_initialize;
};


/**
 * The controller class inheriting Thread
 */
class Controller: public Thread
{
  public:
    
    /**
     * Constructor
     */
    Controller(void (*callback)(void) = NULL, unsigned long _interval = 0);

    /**
     * Start the robot
     */
    void start();

  private:

    /**
     * Internal variable for the current state
     */
    State _state = s_initialize;

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
