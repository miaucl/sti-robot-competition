/*
  sensor-proximity.h - The proximity sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensor_Proximity_h
#define Sensor_Proximity_h

#include "Arduino.h"
#include "config.h"

/* The number of measurements to use for averaging */
#define MEASUREMENT_COUNT 10

/* The number of measurements to use for calibration */
#define CALIBRATION_COUNT 32

/**
 * The sensors class inheriting Thread and Module
 * It detects the proximity value for a sensors
 */
class SensorProximity
{
  public:
    /**
     * Constructor
     */
     SensorProximity( unsigned int sensorId,
                      unsigned int pin,
                      unsigned int threshold = 0);
    /**
     * Exit a state.
     */
    void exitState(State exitState);
    /**
     * Enter a state.
     */
    void enterState(State enterState);
    /**
     * Calibration function of the Sensor.
     */
    void calibrate();
    /**
     * Getter for the sensor id.
     */
    unsigned int getSensorId();
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
    unsigned int threshold = 0;
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
     * Internal variable for the current state
     */
    State _state = s_initialize;

    /**
     * Internal variable for the sensor id
     */
    unsigned int _sensorId;

    /**
     * Internal variable for the timestamp of last runned
     */
    unsigned long _lastRunned;

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
};


#endif
