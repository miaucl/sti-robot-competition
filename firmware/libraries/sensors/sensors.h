/*
  sensors.h - Sensor functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Sensors_h
#define Sensors_h

/**
 * Configure a proximity sensor
 */
 void configureProximity(int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                         int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                         int id,
                         int pin);

/**
 * Calibrate a proximity sensor
 */
 void calibrateProximity(int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                         int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                         int id,
                         int pin);

/**
 * Read proximity sensor values
 */
void readProximity( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                    int *proximityMeasurementIndex,
                    int id,
                    int pin);

/**
 * Get the average value
 */
int getAverageProximityValue( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                              int id);


/**
 * Check if the proximity value is over the threshold
 */
boolean checkProximityThreshold(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                                int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                                int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                                int id);

/**
 * Configure a tof sensor
 */
void configureTOF(int id,
                  int triggerPin,
                  int echoPin);

/**
 * Calibrate a tof sensor
 */
void calibrateTOF(int id,
                  int triggerPin,
                  int echoPin);


/**
 * Read tof sensor values
 */
void readTOF( int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
              int tofMeasurementIndex[SENSOR_TOF_COUNT],
              int id,
              int triggerPin,
              int echoPin);

/**
 * Get the median value
 */
int getMedianTOFValue(int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                      int id);


/**
 * Check if the tof value is over the threshold
 */
boolean checkTOFThreshold(int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          int id);

/**
 * Configure the buttons
 */
void configureBtns();

/**
 * Read the buttons states
 */
void readBtns(boolean btnState[BTN_COUNT]);

#endif