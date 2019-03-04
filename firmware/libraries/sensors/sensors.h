/*
  sensors.h - The sensors of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensors_h
#define Sensors_h

#include "Arduino.h"
#include <ThreadController.h>
#include "config.h"
#include "controller.h"

/**
 * Every class registered in the sensors must be inherited by this class Sensor
 * with a calibration function and a sensor id
 */
class Sensor
{
  public:
    /**
     * Function to be called to calibrate a sensor
     * Abstract class
     */
    virtual calibrate() = 0;
    /**
     * Function to be get the sensor id
     * Abstract class
     */
    virtual getSensorId() = 0;
  private:
    /**
     * Internal variable for the sensor id
     */
    unsigned int _sensorId;
};


/**
 * The sensors class inheriting Thread and Module
 * It provides a general access to all the inputs coming from the different
 * sensors and provides getter functionality as well it manages the data
 * aquisition threads.
 */
class Sensors: public Thread, public Module
{
  public:

    /**
     * Constructor
     */
    Sensors(Sensor *sensorProximityList, unsigned int sensorProximityCount,
            void (*callback)(void) = 0,
            unsigned long _interval = 0);
    /**
     * The explicit implementation of the abstract function of the Module to
     * exit a state.
     */
    exitState(State exitState) override;
    /**
     * The explicit implementation of the abstract function of the Module to
     * enter a state.
     */
    enterState(State enterState) override;
    /**
     * Calibrate all the sensors
     */
    void calibrate();

  private:

    /**
     * A list of all registered proximity sensors
     */
    Sensor *_sensorProximityList;
    /**
     * Number of all registered proximity sensors
     */
    unsigned int _sensorProximityCount;

    /**
     * The overrode run function of the Thread class, being called at a fix
     * frequency rate.
     */
    void run() override;
};


#endif
