/*
  sensors.h - The sensors of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensors_h
#define Sensors_h

#include "Arduino.h"
#include <ThreadController.h>
#include "config.h"
#include "module.h"
#include "sensor-proximity.h"



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
    Sensors(SensorProximity *sensorProximityList,
            unsigned int sensorProximityCount,
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
    /**
     * Get proximity sensor value for sensor id
     */
    unsigned int getProximityValue(unsigned int sensorId);
    /**
     * Check if the threshold has been passed and a flag is raise for sensor id
     */
    unsigned int hasProximityFlagRaised(unsigned int sensorId);

  private:

    /**
     * A list of all registered proximity sensors
     */
    SensorProximity *_sensorProximityList;
    /**
     * Number of all registered proximity sensors
     */
    unsigned int _sensorProximityCount;
    /**
     * Get proximity sensor for sensor id
     */
    SensorProximity* _getProximitySensor(unsigned int sensorId);

    /**
     * The overrode run function of the Thread class, being called at a fix
     * frequency rate.
     */
    void run() override;
};


#endif
