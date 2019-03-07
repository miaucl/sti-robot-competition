/*
  sensor-time-of-flight.h - The time-of-flight sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensor_TimeOfFlight_h
#define Sensor_TimeOfFlight_h

#include "Arduino.h"
#include "config.h"

/* The number of measurements to use for averaging */
#define MEASUREMENT_COUNT 5

/* The max distance the sensor can (should) mesure */
#define MEASUREMENT_MAX_DISTANCE 180

/**
 * The sensors class inheriting Thread and Module
 * It detects the time-of-flight value for a sensors
 */
class SensorTimeOfFlight
{
  public:
    /**
     * Constructor
     */
     SensorTimeOfFlight(unsigned int sensorId,
                        unsigned int triggerPin,
                        unsigned int echoPin,
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
    /**
     * The run function executed asap
     */
    void loop();

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
     * Internal variable for the trigger pin to send the signal
     */
    unsigned int _triggerPin;

    /**
     * Internal variable for the echo pin to read the signal
     */
    unsigned int _echoPin;

    /**
     * Flag for the interrupt that a signal should come
     */
    boolean _waitingForEcho = false;

    /**
     * The millis read for the trigger
     */
    unsigned long _triggerMillis = 0;

    /**
     * The millis read for the echo in the hardware interrupt
     */
    unsigned long _echoMillis = 0;

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
