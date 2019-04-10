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

  return (averageMeasurement > proximityAmbientMeasurements[id] + proximityAmbientVarianceMeasurements[id] + SENSOR_PROXIMITY_THRESHOLD ||
          averageMeasurement < proximityAmbientMeasurements[id] - proximityAmbientVarianceMeasurements[id] - SENSOR_PROXIMITY_THRESHOLD);
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

int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}


// ================================================================
// ===                        MPU Math                          ===
// ================================================================

void MPUMath(uint8_t fifoBuffer[64])
{
  float Yaw, Pitch, Roll;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
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

void calibrateIMU()
{
  // Wait until the IMU has stabelized
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'IMU'");
  #endif

  // long startCalibrationTime = millis();
  // while (millis() - startCalibrationTime < 20000)
  // {
  //   static unsigned long LastGoodPacketTime;
  //   mpuInterrupt = false;
  //   FifoAlive = 1;
  //   fifoCount = mpu.getFIFOCount();
  //   // we have failed Reset and wait till next time!
  //   if ((!fifoCount) || (fifoCount % packetSize))
  //   {
  //     // clear the buffer and start over
  //     mpu.resetFIFO();
  //   }
  //   else
  //   {
  //     #ifdef SERIAL_ENABLE
  //     //Serial.print(".");
  //     Serial.println((millis() - startCalibrationTime) / 1000);
  //     #endif
  //
  //     // Get the packets until we have the latest!
  //     while (fifoCount  >= packetSize)
  //     {
  //       // lets do the magic and get the data
  //       mpu.getFIFOBytes(fifoBuffer, packetSize);
  //       fifoCount -= packetSize;
  //     }
  //     LastGoodPacketTime = millis();
  //     // MPUMath(fifoBuffer); // <<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<
  //     //
  //     // #ifdef SERIAL_ENABLE
  //     // Serial.print("ypr\t");
  //     // Serial.print(ypr[0]);
  //     // Serial.print("\t");
  //     // Serial.print(ypr[1]);
  //     // Serial.print("\t");
  //     // Serial.println(ypr[2]);
  //     // #endif
  //   }
  // }


  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT;)
  {
    static unsigned long LastGoodPacketTime;
    mpuInterrupt = false;
    FifoAlive = 1;
    fifoCount = mpu.getFIFOCount();
    // we have failed Reset and wait till next time!
    if ((!fifoCount) || (fifoCount % packetSize))
    {
      // clear the buffer and start over
      mpu.resetFIFO();
    }
    else
    {
      #ifdef SERIAL_ENABLE
      Serial.print(".");
      #endif

      // Get the packets until we have the latest!
      while (fifoCount  >= packetSize)
      {
        // lets do the magic and get the data
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }
      LastGoodPacketTime = millis();
      MPUMath(fifoBuffer); // <<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<

      // #ifdef SERIAL_ENABLE
      // Serial.print("ypr\t");
      // Serial.print(ypr[0]);
      // Serial.print("\t");
      // Serial.print(ypr[1]);
      // Serial.print("\t");
      // Serial.println(ypr[2]);
      // #endif

      i++;
    }
  }

  #ifdef SERIAL_ENABLE
  Serial.println(" Done!");
  #endif
}


void readIMU( float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
              int *imuMeasurementIndex)
{
  //Serial.print(">");
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  //Serial.print("f");
  // we have failed Reset and wait till next time!
  if ((!fifoCount) || (fifoCount % packetSize))
  {
    // clear the buffer and start over
    mpu.resetFIFO();
    //Serial.println("r");
  }
  else
  {
    // Get the packets until we have the latest!
    while (fifoCount >= packetSize)
    {
      // lets do the magic and get the data
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }
    //Serial.print("p");
    LastGoodPacketTime = millis();
    MPUMath(fifoBuffer); // <<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<
    //Serial.print("m");
  }


  // #ifdef SERIAL_ENABLE
  // Serial.print("ypr\t");
  // Serial.print(ypr[0]);
  // Serial.print("\t");
  // Serial.print(ypr[1]);
  // Serial.print("\t");
  // Serial.println(ypr[2]);
  // #endif

  imuMeasurements[SENSOR_IMU_YAW][*imuMeasurementIndex] = ypr[0] * 180.f/M_PI;
  imuMeasurements[SENSOR_IMU_PITCH][*imuMeasurementIndex] = ypr[1] * 180.f/M_PI;
  imuMeasurements[SENSOR_IMU_ROLL][*imuMeasurementIndex] = ypr[2] * 180.f/M_PI;
  //Serial.print("i");

  // #ifdef SERIAL_ENABLE
  // Serial.print("yprs\t");
  // Serial.print(imuMeasurements[SENSOR_IMU_YAW][*imuMeasurementIndex]);
  // Serial.print("\t");
  // Serial.print(imuMeasurements[SENSOR_IMU_PITCH][*imuMeasurementIndex]);
  // Serial.print("\t");
  // Serial.println(imuMeasurements[SENSOR_IMU_ROLL][*imuMeasurementIndex]);
  // #endif

  (*imuMeasurementIndex)++;
  if (*imuMeasurementIndex >= SENSOR_IMU_MEASUREMENT_COUNT)
  {
    *imuMeasurementIndex = 0;
  }
  //Serial.println(";");

}


/**
 * Get the median value for the z orientation
 */
float getMedianIMUZOrientationValue(float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT])
{
  // Calculate the median of the past measurements

  // Copy values
  float sortedMeasurements[SENSOR_IMU_MEASUREMENT_COUNT] = {0};
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT; i++)
  {
    sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_YAW][i];
    //sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_PITCH][i];
    //sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_ROLL][i];
  }

  // Sort values
  int sortedMeasurementsLength = sizeof(sortedMeasurements) / sizeof(sortedMeasurements[0]);
  qsort(sortedMeasurements, sortedMeasurementsLength, sizeof(sortedMeasurements[0]), sort_asc);


  // Take the middle part and average
  float medianMeasurement = 0;
  for (int i = SENSOR_IMU_MEASUREMENT_COUNT/3; i<SENSOR_IMU_MEASUREMENT_COUNT - (SENSOR_IMU_MEASUREMENT_COUNT/3); i++)
  {
    medianMeasurement += sortedMeasurements[i];
  }
  medianMeasurement /= SENSOR_IMU_MEASUREMENT_COUNT - (2 * (SENSOR_IMU_MEASUREMENT_COUNT/3));
  return medianMeasurement;
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
