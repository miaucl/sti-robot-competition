/*
  controller.h - The controller of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Controller_h
#define Controller_h

#include "Arduino.h"
#include "config.h"
#include "sensor-proximity.h"
#include "sensor-time-of-flight.h"


/**
 * The controller class
 */
class Controller
{
  public:

    /**
     * Constructor
     */
    Controller(SensorProximity *sensorProximityList, SensorTimeOfFlight *sensorTimeOfFlightList);

    /**
     * Start the robot
     */
    void start();

    /**
     * Get the current state
     */
    State getState();

    /**
     * Get the last time the it has runned
     */
    unsigned long getLastRunned();
    /**
     * The run function depending on the period
     */
    void run(unsigned long now);

  private:

    /**
     * The last time it runned
     */
    unsigned long _lastRunned = 0;

    /**
     * Internal variable for the current state
     */
    State _state = s_initialize;

    /**
     * Logging function used to log repeatedly values
     */
    void logging();

    /**
     * Make a state transition from current _state to newState
     */
    void setState(State newState);

    /**
     * The proximity sensors
     */
    SensorProximity *_sensorProximityList;

    /**
     * The time-of-flight sensors
     */
    SensorTimeOfFlight *_sensorTimeOfFlightList;
};

#endif
