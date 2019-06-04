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


// Get the filtered average value
int getFilteredAverageTOFValue( int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                                int id)
{
  float value = 0.f;
  int count = 0;
  for (int i = 0; i<SENSOR_TOF_MEASUREMENT_COUNT; i++)
  {
    if (tofMeasurements[id][i] > 0)
    {
      value += tofMeasurements[id][i];
      count++;
    }
  }

  if (count > 1)
  {
    value /= (float)count;
  }
  return value;
}

// Check if the tof value is over the threshold
boolean checkTOFThreshold(int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT],
                          int id)
{
  int medianMeasurement = getFilteredAverageTOFValue(tofMeasurements, id);

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
bool dmpReady = false;  // set true if DMP init was successful
bool successfulRead = false; // whether a successful read has happened
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t MaxPackets = 20; // Max allowed packets
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

volatile int mpuInterrupt = 0;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt++;
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


// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { // Best version I have made so far
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  //Serial.println(fifoCount);
  // we have failed Reset and wait till next time!
  if ((fifoCount % packetSize) || (fifoCount > (packetSize * MaxPackets)) || (fifoCount < packetSize))
  {
    // clear the buffer and start over
    mpu.resetFIFO();
    // Did not work
    successfulRead = false;
  }
  else
  {
    while (fifoCount  >= packetSize)
    { // Get the packets until we have the latest!
      if (fifoCount < packetSize) return; // Something is left over and we don't want it!!!
      //Serial.print("_");
      // lets do the magic and get the data
      mpu.getFIFOBytes(fifoBuffer, packetSize); // <====== FUCK YOU
      fifoCount -= packetSize;
      successfulRead = true;
    }
    LastGoodPacketTime = millis();
    // Got new values
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
    MPUMath(fifoBuffer);
    // Reset when done and leftovers
    if (fifoCount > 0) mpu.resetFIFO();
  }
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

  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same

  // load and configure the DMP
  devStatus = mpu.dmpInitialize(); // same

  if (devStatus != 0)
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    #ifdef SERIAL_ENABLE
    Serial.print("MPU connection Try #'");
    Serial.print(MPUInitCntr);
    Serial.print("' DMP Initialization failed (code ");
    Serial.print(StatStr[devStatus]);
    Serial.println(")");
    #endif


    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    configureIMU(i2cDataPin, i2cClockPin, interruptPin); // Lets try again
    return;
  }


  // turn on the DMP, now that it's ready
  #ifdef SERIAL_ENABLE
  Serial.println(">Enabling DMP …");
  #endif
  mpu.setDMPEnabled(true);


  // enable Arduino interrupt detection
  #ifdef SERIAL_ENABLE
  Serial.println("Enabling interrupt detection (Arduino external interrupt pin 2)..." );
  Serial.print(">mpu.getInterruptDrive=");
  Serial.println(mpu.getInterruptDrive());
  #endif

  // Attach the interrupt
  attachInterrupt(digitalPinToInterrupt(interruptPin), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus(); // Same

  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  // Max 20 packets
  MaxPackets = 20;
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
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


  // for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT;)
  // {
  //   static unsigned long LastGoodPacketTime;
  //   mpuInterrupt = 0;
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
  //     Serial.print(".");
  //     #endif
  //
  //     // Get the packets until we have the latest!
  //     while (fifoCount >= packetSize)
  //     {
  //       // lets do the magic and get the data
  //       mpu.getFIFOBytes(fifoBuffer, packetSize);
  //       fifoCount -= packetSize;
  //     }
  //     LastGoodPacketTime = millis();
  //     MPUMath(fifoBuffer); // <<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<
  //
  //     // #ifdef SERIAL_ENABLE
  //     // Serial.print("ypr\t");
  //     // Serial.print(ypr[0]);
  //     // Serial.print("\t");
  //     // Serial.print(ypr[1]);
  //     // Serial.print("\t");
  //     // Serial.println(ypr[2]);
  //     // #endif
  //
  //     i++;
  //   }
  // }

  #ifdef SERIAL_ENABLE
  Serial.println(" Done!");
  #endif
}


void readIMU( float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT],
              int *imuMeasurementIndex)
{
  if (!mpuInterrupt) return;

  // Wait until values have been read
  int tryCount = 0;
  //Serial.print("s");
  GetDMP();
  while (!successfulRead)
  {
    //Serial.print("-");
    tryCount++;
    delay(5); // Wait a bit
    GetDMP();

    if (tryCount > 8) // Don't get stuck here
    {
      #ifdef SERIAL_ENABLE
      Serial.println("IMU read failed…");
      #endif
      //delay(1);
      break;
    }
  }
  //Serial.print("e");


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
  // Serial.println(";");

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

  // Check for overflow
  boolean overflow = false;
  int overflowIndex = 0;
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT - 1; i++)
  {
    // Too big jump means overflow
    if (fabsf(sortedMeasurements[i] - sortedMeasurements[i + 1]) > 180.f)
    {
      overflow = true;
      overflowIndex = i;
      break;
    }
  }

  int medianOffset = 0;
  int medianLength = SENSOR_IMU_MEASUREMENT_COUNT;

  // Use the part with more values
  if (overflow)
  {
    if (overflowIndex > SENSOR_IMU_MEASUREMENT_COUNT / 2)
    {
      medianOffset = overflowIndex;
      medianLength = SENSOR_IMU_MEASUREMENT_COUNT - medianOffset;
    }
    else
    {
      medianOffset = 0;
      medianLength = overflowIndex + 1;
    }
  }


  // Take the middle part and average
  float medianMeasurement = 0;
  for (int i = medianLength/3; i<medianLength - (medianLength/3); i++)
  {
    medianMeasurement += sortedMeasurements[medianOffset + i];
  }
  medianMeasurement /= medianLength - (2 * (medianLength/3));

  // Compensate for drift
  medianMeasurement -= SENSOR_IMU_YAW_DRIFT * millis();
  medianMeasurement = wrapPI(medianMeasurement);

  return medianMeasurement;
}


/**
 * Get the median value for the pitch orientation
 */
float getMedianIMUPitchOrientationValue(float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT])
{
  // Calculate the median of the past measurements

  // Copy values
  float sortedMeasurements[SENSOR_IMU_MEASUREMENT_COUNT] = {0};
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT; i++)
  {
    //sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_YAW][i];
    sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_PITCH][i];
    //sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_ROLL][i];
  }

  // Sort values
  int sortedMeasurementsLength = sizeof(sortedMeasurements) / sizeof(sortedMeasurements[0]);
  qsort(sortedMeasurements, sortedMeasurementsLength, sizeof(sortedMeasurements[0]), sort_asc);

  // Check for overflow
  boolean overflow = false;
  int overflowIndex = 0;
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT - 1; i++)
  {
    // Too big jump means overflow
    if (fabsf(sortedMeasurements[i] - sortedMeasurements[i + 1]) > 180.f)
    {
      overflow = true;
      overflowIndex = i;
      break;
    }
  }

  int medianOffset = 0;
  int medianLength = SENSOR_IMU_MEASUREMENT_COUNT;

  // Use the part with more values
  if (overflow)
  {
    if (overflowIndex > SENSOR_IMU_MEASUREMENT_COUNT / 2)
    {
      medianOffset = overflowIndex;
      medianLength = SENSOR_IMU_MEASUREMENT_COUNT - medianOffset;
    }
    else
    {
      medianOffset = 0;
      medianLength = overflowIndex + 1;
    }
  }


  // Take the middle part and average
  float medianMeasurement = 0;
  for (int i = medianLength/3; i<medianLength - (medianLength/3); i++)
  {
    medianMeasurement += sortedMeasurements[medianOffset + i];
  }
  medianMeasurement /= medianLength - (2 * (medianLength/3));

  return medianMeasurement;
}

/**
 * Get the median value for the roll orientation
 */
float getMedianIMURollOrientationValue(float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT])
{
  // Calculate the median of the past measurements

  // Copy values
  float sortedMeasurements[SENSOR_IMU_MEASUREMENT_COUNT] = {0};
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT; i++)
  {
    //sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_YAW][i];
    //sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_PITCH][i];
    sortedMeasurements[i] = imuMeasurements[SENSOR_IMU_ROLL][i];
  }

  // Sort values
  int sortedMeasurementsLength = sizeof(sortedMeasurements) / sizeof(sortedMeasurements[0]);
  qsort(sortedMeasurements, sortedMeasurementsLength, sizeof(sortedMeasurements[0]), sort_asc);

  // Check for overflow
  boolean overflow = false;
  int overflowIndex = 0;
  for (int i = 0; i<SENSOR_IMU_MEASUREMENT_COUNT - 1; i++)
  {
    // Too big jump means overflow
    if (fabsf(sortedMeasurements[i] - sortedMeasurements[i + 1]) > 180.f)
    {
      overflow = true;
      overflowIndex = i;
      break;
    }
  }

  int medianOffset = 0;
  int medianLength = SENSOR_IMU_MEASUREMENT_COUNT;

  // Use the part with more values
  if (overflow)
  {
    if (overflowIndex > SENSOR_IMU_MEASUREMENT_COUNT / 2)
    {
      medianOffset = overflowIndex;
      medianLength = SENSOR_IMU_MEASUREMENT_COUNT - medianOffset;
    }
    else
    {
      medianOffset = 0;
      medianLength = overflowIndex + 1;
    }
  }


  // Take the middle part and average
  float medianMeasurement = 0;
  for (int i = medianLength/3; i<medianLength - (medianLength/3); i++)
  {
    medianMeasurement += sortedMeasurements[medianOffset + i];
  }
  medianMeasurement /= medianLength - (2 * (medianLength/3));

  return medianMeasurement;
}






/***********/
/* Buttons */
/***********/


void configureBtns()
{
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  pinMode(BTN_STATE_PIN, INPUT_PULLUP);
  pinMode(BTN_EEPROM_PIN, INPUT_PULLUP);
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
}

// Read the button states
void readBtns(boolean btnState[BTN_COUNT])
{
  btnState[BTN_START] = (digitalRead(BTN_START_PIN) == LOW);
  btnState[BTN_STATE] = (digitalRead(BTN_STATE_PIN) == LOW);
  btnState[BTN_EEPROM] = (digitalRead(BTN_EEPROM_PIN) == LOW);
  btnState[BTN_MODE] = (digitalRead(BTN_MODE_PIN) == LOW);
}
