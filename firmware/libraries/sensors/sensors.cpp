/*
  utils.cpp - Utility functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"
#include "utils.h"

#include "MPU6050_6Axis_MotionApps20.h"


/*************/
/* Proximity */
/*************/

void configureProximity(int id,
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

  // Sort values
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


/*******/
/* IMU */
/*******/

/**
 * The IMU sensor object
 */
MPU6050 mpu;


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}


void configureIMU(int i2cDataPin,
                  int i2cClockPin,
                  int interruptPin)
{
  // Initialize and configure imu device
  #ifdef SERIAL_ENABLE
  Serial.print("Configure Sensor 'IMU' on interrupt pin '");
  Serial.print(interruptPin);
  Serial.println("'");
  #endif
  mpu.initialize();

  // Verify connection
  #ifdef SERIAL_ENABLE
  Serial.print(">Testing imu sensor connection > ");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  #else
  mpu.testConnection();
  #endif

  // Load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // Wait a bit
  delay(200);

  /*
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  */

  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    #ifdef SERIAL_ENABLE
    Serial.println(">Enabling DMP …");
    #endif
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    #ifdef SERIAL_ENABLE
    Serial.println(">Enabling interrupt detection (Arduino external interrupt 0) …");
    #endif
    attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    #ifdef SERIAL_ENABLE
    Serial.print("#### DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    #endif
  }
}

void calibrateIMU(float imuOffsetMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS])
{
  Quaternion q;
  VectorFloat gravity;
  float ypr[SENSOR_IMU_MEASUREMENT_DIMENSIONS];


  // Wait until the IMU has stabelized
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrating");
  #endif

  for (int i = 0; i<10; i++)
  {
    #ifdef SERIAL_ENABLE
    Serial.print(".");
    #endif
    delay(300);
  }
  #ifdef SERIAL_ENABLE
  Serial.println(" Done!");
  #endif

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT;)
  {
    // Wait for data
    while (!mpuInterrupt && fifoCount < packetSize);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    // May happen due to other setup, clear and retry
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      #ifdef SERIAL_ENABLE
      Serial.println("FIFO overflow > Clean and retry …");
      #endif
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display YPR angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      for (int j = 0; j<SENSOR_IMU_MEASUREMENT_DIMENSIONS; j++)
      {
        imuOffsetMeasurements[j] += ypr[j] * 180/M_PI;
      }

      #ifdef SERIAL_ENABLE
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
      #endif

      i++;
    }
  }

  // Calculate the offset
  for (int j = 0; j<SENSOR_IMU_MEASUREMENT_DIMENSIONS; j++)
  {
    imuOffsetMeasurements[j] /= SENSOR_IMU_MEASUREMENT_COUNT;
  }

  #ifdef SERIAL_ENABLE
  Serial.print("ypr offset\t");
  Serial.print(imuOffsetMeasurements[0]);
  Serial.print("\t");
  Serial.print(imuOffsetMeasurements[1]);
  Serial.print("\t");
  Serial.println(imuOffsetMeasurements[2]);
  #endif


  // Reset the FIFO
  mpu.resetFIFO();
}


void readIMU( float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
              int *imuMeasurementIndex)
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // Wait for data
  if (!mpuInterrupt && fifoCount < packetSize) return;

  Quaternion q;
  VectorFloat gravity;
  float ypr[3];

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    #ifdef SERIAL_ENABLE
    Serial.println("FIFO overflow > Clean and retry …");
    #endif
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // display YPR angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // #ifdef SERIAL_ENABLE
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180/M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180/M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[2] * 180/M_PI);
    // #endif

    // Save values
    imuMeasurements[SENSOR_IMU_YAW][*imuMeasurementIndex] = ypr[0] * 180/M_PI;
    imuMeasurements[SENSOR_IMU_PITCH][*imuMeasurementIndex] = ypr[1] * 180/M_PI;
    imuMeasurements[SENSOR_IMU_ROLL][*imuMeasurementIndex] = ypr[2] * 180/M_PI;
    (*imuMeasurementIndex)++;
    if (*imuMeasurementIndex >= SENSOR_TOF_MEASUREMENT_COUNT)
    {
      *imuMeasurementIndex = 0;
    }

    // reset FIFO buffer, as there are more packets available than they can be processed
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
  }
}

/**
 * Get the median value for the z orientation
 */
int getMedianIMUZOrientationValue(int imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT])
{
  // Calculate the median of the past measurements

  // Copy values
  int sortedMeasurements[SENSOR_IMU_MEASUREMENT_COUNT] = {0};
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT; i++)
  {
    sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_YAW][i];
  }

  // Sort values
  int sortedMeasurementsLength = sizeof(sortedMeasurements) / sizeof(sortedMeasurements[0]);
  qsort(sortedMeasurements, sortedMeasurementsLength, sizeof(sortedMeasurements[0]), sort_asc);


  // Take the middle part and average
  int medianMeasurement = 0;
  for (int i = SENSOR_IMU_MEASUREMENT_COUNT/3; i<SENSOR_IMU_MEASUREMENT_COUNT - (SENSOR_IMU_MEASUREMENT_COUNT/3); i++)
  {
    medianMeasurement += sortedMeasurements[i];
  }
  medianMeasurement /= SENSOR_IMU_MEASUREMENT_COUNT - (2 * (SENSOR_IMU_MEASUREMENT_COUNT/3));
  return medianMeasurement;
}


/**
 * Check if the z orientation value is over the threshold
 */
boolean checkIMUZOrientationThreshold(int imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT])
{
  int medianMeasurement = getMedianIMUZOrientationValue(imuMeasurements);

  return (abs(medianMeasurement) < SENSOR_IMU_Z_ORIENTATION_THRESHOLD);
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
