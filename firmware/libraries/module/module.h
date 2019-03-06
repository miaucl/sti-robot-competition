/*
  module.h - The modules of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Module_h
#define Module_h

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

#endif
