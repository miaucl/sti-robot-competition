/*
  sensor-proximity.h - The proximity sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensor_Proximity_h
#define Sensor_Proximity_h

#include "Arduino.h"
#include <Thread.h>
#include "config.h"
#include "sensor.h"
#include "module.h"

/* The number of measurements to use for averaging */
#define MEASUREMENT_COUNT 10

/* The number of measurements to use for calibration */
#define CALIBRATION_COUNT 32

#define FLAG_THRESHOLD 50

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
                      unsigned int theshold = 0,
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
    /**
     * Get the most recent sensor value
     */
    unsigned int getValue();
    /**
     * Check if the threshold has been passed and a flag is raised
     */
    boolean hasFlagRaised();
    /**
     * The the threshold, by which the value has to raise to rais a flag
     * VALUE > _ambient + _ambientVariance + theshold
     */
    unsigned int theshold = 0;

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
     * The ambient light determined at calibration
     */
    long _ambient;

    /**
     * The variance ambient light determined at calibration
     */
    long _ambientVariance;

    /**
     * Internal array for averaging measurements
     */
    unsigned int _measurements[MEASUREMENT_COUNT] = {0};

    /**
     * Index for internal array for averaging measurements
     */
    unsigned int _measurementIndex = 0;

    /**
     * The overrode run function of the Thread class, being called at a fix
     * frequency rate.
     */
    void run() override;
};


#endif
