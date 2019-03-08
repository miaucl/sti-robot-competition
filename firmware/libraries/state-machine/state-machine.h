/*
  state-machine.h - State machine functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef StateMachine_h
#define StateMachine_h

#include "config.h"

/**
 * Check if a transition to a new state is required
 * Returns the same state if not
 */
State checkStateTransition( State currentState,
                            long stateChangeTimestamp,
                            boolean btnState[BTN_COUNT]);

#endif
