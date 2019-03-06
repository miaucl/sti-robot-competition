/*
  sensor.h - The sensor of the robot
  Created by Cyrill Lippuner, 2019.
*/

#ifndef Sensor_h
#define Sensor_h


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

#endif
