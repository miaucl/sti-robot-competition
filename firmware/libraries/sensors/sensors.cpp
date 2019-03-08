/*
  utils.cpp - Utility functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"
#include "utils.h"

/*************/
/* Proximity */
/*************/

void configureProximity(int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int id,
                        int pin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Configure Sensor 'Proximity ");
  Serial.print(id);
  Serial.print("' on pin '");
  Serial.print(pin);
  Serial.println("'");
  #endif
}

void calibrateProximity(int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                        int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                        int id,
                        int pin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'Proximity ");
  Serial.print(id);
  Serial.println("'");
  #endif

  // Get a measure of the ambient light for the sensor
  int ambient = 0;
  int ambientVariance = 0;
  int values[SENSOR_PROXIMITY_CALIBRATION_COUNT];
  for (unsigned int i = 0; i<SENSOR_PROXIMITY_CALIBRATION_COUNT; i++)
  {
    values[i] = analogRead(pin);
    ambient += values[i];
    delay(5); // Wait for 5 milliseconds between each measurement
  }
  ambient /= SENSOR_PROXIMITY_CALIBRATION_COUNT;
  proximityAmbientMeasurements[id] = ambient;

  for (unsigned int i = 0; i<SENSOR_PROXIMITY_CALIBRATION_COUNT; i++)
  {
    ambientVariance += abs(values[i] - ambient);
  }
  ambientVariance /= SENSOR_PROXIMITY_CALIBRATION_COUNT;
  ambientVariance *= 2;

  proximityAmbientVarianceMeasurements[id] = ambientVariance;
}

void readProximity( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                    int proximityMeasurementIndex[SENSOR_PROXIMITY_COUNT],
                    int id,
                    int pin)
{
  // Read the value from the analog sensor
  int measurement = analogRead(pin);

  // Save the measurement
  proximityMeasurements[id][proximityMeasurementIndex[id]++] = measurement;
  if (proximityMeasurementIndex[id] >= SENSOR_PROXIMITY_MEASUREMENT_COUNT)
  {
    proximityMeasurementIndex[id] = 0;
  }
}

int getAverageProximityValue( int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                              int id)
{
  // Calculate the average of the past measurements
  int averageMeasurement = 0;
  for (int i = 0; i<SENSOR_PROXIMITY_MEASUREMENT_COUNT; i++)
  {
    averageMeasurement += proximityMeasurements[id][i];
  }
  averageMeasurement /= SENSOR_PROXIMITY_MEASUREMENT_COUNT;

  return averageMeasurement;
}

boolean checkProximityThreshold(int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT],
                                int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT],
                                int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT],
                                int id)
{
  int averageMeasurement = getAverageProximityValue(proximityMeasurements, id);

  return (averageMeasurement > proximityAmbientMeasurements[id] + proximityAmbientVarianceMeasurements[id] + SENSOR_PROXIMITY_THRESHOLD);
}

/*******/
/* TOF */
/*******/

void configureTOF(int id,
                  int triggerPin,
                  int echoPin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Configure Sensor 'TOF ");
  Serial.print(id);
  Serial.print("' on pins '");
  Serial.print(triggerPin);
  Serial.print(", ");
  Serial.print(echoPin);
  Serial.println("'");
  #endif

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void calibrateTOF(int id,
                  int triggerPin,
                  int echoPin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'TOF ");
  Serial.print(id);
  Serial.println("'");
  #endif
}


// Read tof sensor values
void readTOF( int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
              int tofMeasurementIndex[SENSOR_TOF_COUNT],
              int id,
              int triggerPin,
              int echoPin)
{
  // Send a new ping
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH, SENSOR_TOF_TIMEOUT);
  long measurement = (duration/2) / SENSOR_TOF_CONVERTION_FACTOR;

  tofMeasurements[id][tofMeasurementIndex[id]++] = measurement;
  if (tofMeasurementIndex[id] >= SENSOR_TOF_MEASUREMENT_COUNT)
  {
    tofMeasurementIndex[id] = 0;
  }
}


// Get the median value
int getMedianTOFValue(int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                      int id)
{
  // Calculate the median of the past measurements

  // Copy values
  int sortedMeasurements[SENSOR_TOF_MEASUREMENT_COUNT] = {0};
  for (int i = 0; i<SENSOR_TOF_MEASUREMENT_COUNT; i++)
  {
    sortedMeasurements[i] = tofMeasurements[id][i];
  }

  // Sort valies
  int sortedMeasurementsLength = sizeof(sortedMeasurements) / sizeof(sortedMeasurements[0]);
  qsort(sortedMeasurements, sortedMeasurementsLength, sizeof(sortedMeasurements[0]), sort_asc);


  // Take the middle part and average
  int medianMeasurement = 0;
  for (int i = SENSOR_TOF_MEASUREMENT_COUNT/3; i<SENSOR_TOF_MEASUREMENT_COUNT - (SENSOR_TOF_MEASUREMENT_COUNT/3); i++)
  {
    medianMeasurement += sortedMeasurements[i];
  }
  medianMeasurement /= SENSOR_TOF_MEASUREMENT_COUNT - (2 * (SENSOR_TOF_MEASUREMENT_COUNT/3));
  return medianMeasurement;
}

// Check if the tof value is over the threshold
boolean checkTOFThreshold(int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          int id)
{
  int medianMeasurement = getMedianTOFValue(tofMeasurements, id);

  return (medianMeasurement > 0);
}

/***********/
/* Buttons */
/***********/


void configureBtns()
{
  pinMode(BTN_START_PIN, INPUT_PULLUP);
}

// Read the button states
void readBtns(boolean btnState[BTN_COUNT])
{
  btnState[BTN_START] = (digitalRead(BTN_START_PIN) == LOW);
}
