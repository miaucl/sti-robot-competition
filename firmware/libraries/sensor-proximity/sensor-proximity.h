/*
  sensor-proximity.h - The proximity sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensor_Proximity_h
#define Sensor_Proximity_h

#include "Arduino.h"
#include <Thread.h>
#include "config.h"
#include "sensors.h"
#include "controller.h"


/**
 * The sensors class inheriting Thread and Module
 * It detects the proximity value for a sensors
 */
class SensorProximity: public Thread, public Module, public Sensor
{
  public:
    /**
     * Constructor
     */
     SensorProximity( unsigned int sensorId,
                      unsigned int pin,
                      void (*callback)(void) = NULL,
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
     * The explicit implementation of the calibration function of the Sensor.
     */
    calibrate() override;
    /**
     * The explicit implementation of the getter for the sensor id.
     */
    getSensorId() override;

  private:

    /**
     * Internal variable for the sensor id
     */
    unsigned int _sensorId;

    /**
     * Internal variable for the pin to read the analog value for the proximity
     */
    unsigned int _pin;

    /**
     * The overrode run function of the Thread class, being called at a fix
     * frequency rate.
     */
    void run() override;
};


#endif
