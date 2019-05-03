#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "config.h"
#include "utils.h"
#include "sensors.h"
#include "leds.h"
#include "actuators.h"
#include "state-estimator.h"
#include "state-machine.h"
#include "state-following.h"
#include "state-wander.h"
#include "state-swallowing.h"
#include "state-scanning.h"
#include "state-turning.h"

/*********
 * Global values
 */

// State for the state machine
int state = s_initialization;
long stateChangeTimestamp = millis();

// Global pause
boolean globalPause = false;
boolean nGlobalPauseButtonState = true;

// The measurements for the proximity sensors
int proximityAmbientMeasurements[SENSOR_PROXIMITY_COUNT] = {0};
int proximityAmbientVarianceMeasurements[SENSOR_PROXIMITY_COUNT] = {0};
int proximityMeasurements[SENSOR_PROXIMITY_COUNT][SENSOR_PROXIMITY_MEASUREMENT_COUNT] = {0};
int proximityMeasurementIndex[SENSOR_PROXIMITY_COUNT] = {0};

// The measurements for the tof sensors
int tofMeasurements[SENSOR_TOF_COUNT][SENSOR_TOF_MEASUREMENT_COUNT] = {0};
int tofMeasurementIndex[SENSOR_TOF_COUNT] = {0};

// The measurements for the imu sensors
float imuMeasurements[SENSOR_IMU_MEASUREMENT_DIMENSIONS][SENSOR_IMU_MEASUREMENT_COUNT] = {0};
int imuMeasurementIndex = 0;

// The buttons for control
boolean btnState[BTN_COUNT] = {0};

// The leds for feedback
boolean ledState[LED_COUNT] = {0};

// The control values for the motor actuators and the measurement of the position for each step
double motorSpeeds[ACTUATOR_MOTOR_COUNT] = {0};
double motorSpeedMeasurements[ACTUATOR_MOTOR_COUNT] = {0};

// The angle for the servo actuators
int servoAngles[ACTUATOR_SERVO_COUNT] = {0};

// The flags
boolean flags[FLAG_COUNT] = {0};


// State estimator for position and angle
StateEstimator estimator;
float estimatorAngleOffset = 0;



/*********
 * Initialization
 */
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  #ifdef SERIAL_CONN_ENABLE
  // Setup serial communication
  Serial.begin(SERIAL_BAUD_RATE);
  #endif

  #ifdef SERIAL_ENABLE
  Serial.print("ROBOT ID: ");
  Serial.println(ROBOT_ID);
  Serial.println("Start Up …");
  #endif

  pinMode(GLOBAL_PAUSE_BTN, INPUT_PULLUP);
  pinMode(GLOBAL_PAUSE_LED, OUTPUT);

  // Configuration
  configuration();
}


/**
 * Configuration
 */
void configuration()
{
  #ifdef SERIAL_ENABLE
  Serial.println("Configure Sensors");
  #endif

  configureProximity(SENSOR_PROXIMITY_RIGHT, SENSOR_PROXIMITY_RIGHT_PIN);
  configureProximity(SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN);
  configureProximity(SENSOR_PROXIMITY_FORWARD, SENSOR_PROXIMITY_FORWARD_PIN);
  configureProximity(SENSOR_PROXIMITY_FORWARD_LEFT, SENSOR_PROXIMITY_FORWARD_LEFT_PIN);
  configureProximity(SENSOR_PROXIMITY_LEFT, SENSOR_PROXIMITY_LEFT_PIN);
  configureProximity(SENSOR_PROXIMITY_BACKWARD, SENSOR_PROXIMITY_BACKWARD_PIN);
  configureProximity(SENSOR_PROXIMITY_DOWN_RIGHT, SENSOR_PROXIMITY_DOWN_RIGHT_PIN);
  configureProximity(SENSOR_PROXIMITY_DOWN_LEFT, SENSOR_PROXIMITY_DOWN_LEFT_PIN);
  configureProximity(SENSOR_PROXIMITY_DETECT_RIGHT, SENSOR_PROXIMITY_DETECT_RIGHT_PIN);
  configureProximity(SENSOR_PROXIMITY_DETECT_LEFT, SENSOR_PROXIMITY_DETECT_LEFT_PIN);

  configureTOF(SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);
  configureTOF(SENSOR_TOF_CENTER, SENSOR_TOF_CENTER_TRIGGER_PIN, SENSOR_TOF_CENTER_ECHO_PIN);
  configureTOF(SENSOR_TOF_LEFT, SENSOR_TOF_LEFT_TRIGGER_PIN, SENSOR_TOF_LEFT_ECHO_PIN);

  configureIMU(SENSOR_IMU_SDA_PIN, SENSOR_IMU_SCL_PIN, SENSOR_IMU_INT_PIN);

  configureBtns();

  configureLeds();

  configureMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  configureMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

  configureServo(ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  configureServo(ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

  servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
  servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
  resetServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  resetServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

  Matrix<3> x0 = {0.5,0.5,0};
  estimator.init(x0);
}










/**********
 * Start main loop
 */
void loop()
{
  // Global pause button
  if (nGlobalPauseButtonState != digitalRead(GLOBAL_PAUSE_BTN))
  {
    delay(10);
    nGlobalPauseButtonState = !nGlobalPauseButtonState;

    if (!nGlobalPauseButtonState)
    {
      globalPause = !globalPause;
      digitalWrite(GLOBAL_PAUSE_LED, globalPause);
      Serial.print(">> Global Pause: ");
      Serial.println(globalPause);
    }
  }
  // Return if globally paused
  if (globalPause) return;

  // Wait for period
  static long lastPeriodStart = millis();
  static long dt = 0;
  while (millis() - lastPeriodStart < PERIOD) { }
  dt = millis() - lastPeriodStart;
  lastPeriodStart = millis();





  // Check for state transition
  State nextState = checkStateTransition(state, stateChangeTimestamp, btnState, flags);
  if (state != nextState)
  {
    #ifdef SERIAL_ENABLE
    Serial.print("State Transition: ");
    Serial.print(state);
    Serial.print(" > ");
    Serial.print(nextState);
    Serial.println("");
    #endif


    // Exit previous state
    switch (state)
    {
      case s_initialization: stateInitializationExit(); break;
      case s_calibration: stateCalibrationExit(); break;
      case s_idle: stateIdleExit(); break;
      case s_wander: stateWanderExit(); break;
      case s_test: stateTestExit(); break;
      case s_following: stateFollowingExit(); break;
      case s_swallowing: stateSwallowingExit(); break;
      case s_scanning: stateScanningExit(); break;
      case s_turning: stateTurningExit(); break;
    }

    // Enter next state
    switch (nextState)
    {
      case s_initialization: stateInitializationEnter(); break;
      case s_calibration: stateCalibrationEnter(); break;
      case s_idle: stateIdleEnter(); break;
      case s_wander: stateWanderEnter(); break;
      case s_test: stateTestEnter(); break;
      case s_following: stateTestEnter(); break;
      case s_swallowing: stateSwallowingEnter(); break;
      case s_scanning: stateScanningEnter(); break;
      case s_turning: stateTurningEnter(); break;
    }

    // Set next state
    state = nextState;
    stateChangeTimestamp = millis();
  }

  // Feedback
  writeLeds(ledState);

  switch (state)
  {
    case s_initialization: stateInitialization(); break;
    case s_calibration: stateCalibration(); break;
    case s_idle: stateIdle(); break;
    case s_wander: stateWander(); break;
    case s_test: stateTest(); break;
    case s_following: stateFollowing(); break;
    case s_swallowing: stateSwallowing(); break;
    case s_scanning: stateScanning(); break;
    case s_turning: stateTurning(); break;
  }

  // Update Estimator
  updateEstimator(dt);

  // Feedback
  writeLeds(ledState);


  // Logging
  log();
}

// ================================================================
// ===                        INITIALIZATION STATE              ===
// ================================================================


// The "s_initialization" state s
void stateInitializationEnter()
{
  // Do nothing
}

void stateInitialization()
{
  // Do nothing
}

void stateInitializationExit()
{
  // Do nothing
}


// ================================================================
// ===                        CALIBRATION STATE                 ===
// ================================================================


// The "s_calibration" state
void stateCalibrationEnter()
{
  ledState[LED_SYSTEM] = HIGH; writeLeds(ledState);

  #ifdef SERIAL_ENABLE
  Serial.println("Calibrate Sensors");
  #endif

  delay(200);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);

  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_RIGHT, SENSOR_PROXIMITY_RIGHT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD, SENSOR_PROXIMITY_FORWARD_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT, SENSOR_PROXIMITY_FORWARD_LEFT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_LEFT, SENSOR_PROXIMITY_LEFT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_BACKWARD, SENSOR_PROXIMITY_BACKWARD_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT, SENSOR_PROXIMITY_DOWN_RIGHT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_LEFT, SENSOR_PROXIMITY_DOWN_LEFT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT, SENSOR_PROXIMITY_DETECT_RIGHT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_LEFT, SENSOR_PROXIMITY_DETECT_LEFT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateTOF(SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateTOF(SENSOR_TOF_CENTER, SENSOR_TOF_CENTER_TRIGGER_PIN, SENSOR_TOF_CENTER_ECHO_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateTOF(SENSOR_TOF_LEFT, SENSOR_TOF_LEFT_TRIGGER_PIN, SENSOR_TOF_LEFT_ECHO_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateIMU();
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);

  #ifdef SERIAL_ENABLE
  Serial.println("Calibrate Estimator …");
  #endif
  readIMU(imuMeasurements, &imuMeasurementIndex);
  float calibrateAngle = 0.f;
  int calibrateCounter = 0;
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  for (int i = 0; i<ESTIMATOR_CALIBRATION_MAX; i++) // Wait for IMU to get stabelized
  {
    delay(200); 
    ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
    readIMU(imuMeasurements, &imuMeasurementIndex);
    float newCalibrateAngle = imuMeasurements[SENSOR_IMU_YAW][imuMeasurementIndex];
    if (fabsf(newCalibrateAngle) < 1) continue;

    #ifdef SERIAL_ENABLE
    Serial.print("-");
    #endif
    if (fabsf(newCalibrateAngle - calibrateAngle) > 0.001)
    {
      calibrateAngle = newCalibrateAngle;
    }
    else
    {
      calibrateCounter++;
    }

    if (calibrateCounter > ESTIMATOR_CALIBRATION_COUNTER)
    {
      break;
    }
  }
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  #ifdef SERIAL_ENABLE
  Serial.println(" Done!");
  #endif
  delay(200);
  setEstimatorAngleOffset();
  
  ledState[LED_SYSTEM] = HIGH; writeLeds(ledState);
}

void stateCalibration()
{
  readBtns(btnState);
}

void stateCalibrationExit()
{
  ledState[LED_SYSTEM] = LOW;
}

// ================================================================
// ===                        IDLE STATE                        ===
// ================================================================


// The "s_idle" state
void stateIdleEnter()
{
  flags[FLAG_ENABLE_ESTIMATOR] = false;
  ledState[LED_SYSTEM] = false;
}

void stateIdle()
{
  readBtns(btnState);

  static long t = millis();
  if (millis() - t > 500)
  {
    ledState[LED_SYSTEM] = !ledState[LED_SYSTEM];
    t = millis();
  }
}

void stateIdleExit()
{
  ledState[LED_SYSTEM] = false;
}

// ================================================================
// ===                        TEST STATE                        ===
// ================================================================


// The "s_test" state
void stateTestEnter()
{
  ledState[LED_RUNNING] = HIGH;
  flags[FLAG_ENABLE_ESTIMATOR] = true;
  setEstimatorAngleOffset();
}

void stateTest()
{
  readAll();

  static boolean raw = false;

  // Testing
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    char b = Serial.read();

    if (b == 32)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
    }
    else if (b == 119)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] += 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] += 0.1;
    }
    else if (b == 115)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] -= 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] -= 0.1;
    }
    else if (b == 97)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] += 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] -= 0.1;
    }
    else if (b == 100)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] -= 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] += 0.1;
    }
    else if (b == 111)
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;

    }
    else if (b == 108)
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
    }
    else if (b == 46)
    {
      raw = !raw;
    }

    if (raw)
    {
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeRawMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    else
    {
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
    writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
  }



  updateAll();
//
//  int p = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD);
//  int tl = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);
//  int tc = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
//  int tr = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
//  if (p > 300 && (tl == 0 || tl > 32) && (tc == 0 || tc > 32) && (tr == 0 || tr > 32))
//  {
//    Serial.println("Bottle");
//  }
//  else if (p > 200 && (tl > 0 && tl < 32 || tc > 0 && tc < 32 || tr > 0 && tr < 32))
//  {
//    Serial.println("Wall");
//  }
//  else
//  {
//    Serial.println("–");
//  }
//
Serial.print("Z: ");
Serial.print(getMedianIMUZOrientationValue(imuMeasurements));
Serial.print("\tIR: ");
Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD));
Serial.print("\tTOF: ");
Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT));
Serial.print(", ");
Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER));
Serial.print(", ");
Serial.println(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT));

}

void stateTestExit()
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
  flags[FLAG_ENABLE_ESTIMATOR] = true;
}


// ================================================================
// ===                        WANDER STATE                      ===
// ================================================================


// The "s_wander" state
void stateWanderEnter()
{
  stateWanderEnterRoutine(ledState, flags);
}

void stateWander()
{
  readAll();

  stateWanderRoutine( proximityMeasurements, 
                      proximityAmbientMeasurements, 
                      proximityAmbientVarianceMeasurements, 
                      tofMeasurements, 
                      imuMeasurements, 
                      motorSpeeds, 
                      motorSpeedMeasurements, 
                      btnState, 
                      ledState, 
                      flags);

  updateAll();
}

void stateWanderExit()
{
  stateWanderExitRoutine(motorSpeeds, ledState, flags);
}

// ================================================================
// ===                        FOLLOWING STATE                   ===
// ================================================================


// The "s_following" state
void stateFollowingEnter()
{
  stateFollowingEnterRoutine(ledState, flags);
}

void stateFollowing()
{
  readAll();

  stateFollowingRoutine(proximityMeasurements, 
                        proximityAmbientMeasurements, 
                        proximityAmbientVarianceMeasurements, 
                        tofMeasurements, 
                        imuMeasurements, 
                        motorSpeeds, 
                        motorSpeedMeasurements, 
                        btnState, 
                        ledState, 
                        flags);

  updateAll();
}

void stateFollowingExit()
{
  stateFollowingExitRoutine(ledState, flags);
}



// ================================================================
// ===                       SWALLOWING STATE                   ===
// ================================================================


// The "s_swallowing" state
void stateSwallowingEnter()
{
  stateSwallowingEnterRoutine(ledState, flags);
}

void stateSwallowing()
{
  readAll();

  stateSwallowingRoutine( proximityMeasurements, 
                          proximityAmbientMeasurements, 
                          proximityAmbientVarianceMeasurements, 
                          tofMeasurements, 
                          imuMeasurements, 
                          motorSpeeds, 
                          motorSpeedMeasurements, 
                          servoAngles, 
                          btnState, 
                          ledState, 
                          flags);

  updateAll();
}

void stateSwallowingExit()
{
  stateSwallowingExitRoutine(ledState, flags);
}


// ================================================================
// ===                       SCANNING STATE                     ===
// ================================================================


// The "s_scanning" state
void stateScanningEnter()
{
  stateScanningEnterRoutine(ledState, flags);
}

void stateScanning()
{
  readAll();

  stateScanningRoutine( proximityMeasurements, 
                        proximityAmbientMeasurements, 
                        proximityAmbientVarianceMeasurements, 
                        tofMeasurements, 
                        imuMeasurements, 
                        motorSpeeds, 
                        motorSpeedMeasurements, 
                        btnState, 
                        ledState, 
                        flags);

  updateAll();
}

void stateScanningExit()
{
  stateScanningExitRoutine(ledState, flags);
}

// ================================================================
// ===                       TURNING STATE                      ===
// ================================================================


// The "s_scanning" state
void stateTurningEnter()
{
  stateTurningEnterRoutine(ledState, flags);
}

void stateTurning()
{
  readAll();

  stateTurningRoutine( imuMeasurements, 
                        motorSpeeds, 
                        motorSpeedMeasurements,
                        btnState, 
                        ledState, 
                        flags);

  updateAll();
}

void stateTurningExit()
{
  stateTurningExitRoutine(ledState, flags);
}




/**
 * Read all
 */
void readAll()
{
  readBtns(btnState);

  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_RIGHT, SENSOR_PROXIMITY_RIGHT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_FORWARD, SENSOR_PROXIMITY_FORWARD_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_FORWARD_LEFT, SENSOR_PROXIMITY_FORWARD_LEFT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_LEFT, SENSOR_PROXIMITY_LEFT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_BACKWARD, SENSOR_PROXIMITY_BACKWARD_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_DOWN_RIGHT, SENSOR_PROXIMITY_DOWN_RIGHT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_DOWN_LEFT, SENSOR_PROXIMITY_DOWN_LEFT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_DETECT_RIGHT, SENSOR_PROXIMITY_DETECT_RIGHT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_DETECT_LEFT, SENSOR_PROXIMITY_DETECT_LEFT_PIN);

  // Only look at one tof each time
  static int tofIndex = 0;
  if (tofIndex == 0) readTOF(tofMeasurements, tofMeasurementIndex, SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);
  if (tofIndex == 1) readTOF(tofMeasurements, tofMeasurementIndex, SENSOR_TOF_CENTER, SENSOR_TOF_CENTER_TRIGGER_PIN, SENSOR_TOF_CENTER_ECHO_PIN);
  if (tofIndex == 2) readTOF(tofMeasurements, tofMeasurementIndex, SENSOR_TOF_LEFT, SENSOR_TOF_LEFT_TRIGGER_PIN, SENSOR_TOF_LEFT_ECHO_PIN);
  if (++tofIndex == 3) tofIndex = 0;

  readIMU(imuMeasurements, &imuMeasurementIndex);
}


/**
 * Update all
 */
void updateAll()
{
  updateMotorSpeedControl(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN, motorSpeedMeasurements);
  updateMotorSpeedControl(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN, motorSpeedMeasurements);
  updateServoAngleControl(ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  updateServoAngleControl(ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
}

/**
 * Set angle estimator offset
 */
void setEstimatorAngleOffset()
{
  estimatorAngleOffset = imuMeasurements[SENSOR_IMU_YAW][imuMeasurementIndex];
  #ifdef SERIAL_ENABLE
  Serial.print("Set Estimator Offset: ");
  Serial.println(estimatorAngleOffset);
  #endif
}

/**
 * Upate estimator
 */
void updateEstimator(long dt)
{
  if (flags[FLAG_ENABLE_ESTIMATOR])
  {
    float angle = imuMeasurements[SENSOR_IMU_YAW][imuMeasurementIndex] - estimatorAngleOffset;
    float speed = (motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] + motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) / 2.f;

    estimator.step(speed, -angle, dt / 1000.f);

    Matrix<2> p = estimator.getPosition();

//    Serial.print(p(0));
//    Serial.print(", ");
//    Serial.print(p(1));
//    Serial.print(", ");
//    Serial.println(state);
  }
}


/**
 * Logging
 */
void log()
{
//  Serial.print("S: ");
//  Serial.print(state);
//  Serial.print("\t PROX: ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_RIGHT)) Serial.print("R ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT)) Serial.print("FR ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD)) Serial.print("F ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT)) Serial.print("FL ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_LEFT)) Serial.print("L ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_BACKWARD)) Serial.print("B ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT)) Serial.print("DR ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_LEFT)) Serial.print("DL ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT)) Serial.print("DTR ");
//  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_LEFT)) Serial.print("DTL ");

//  Serial.print("\t TOF: ");
//  if (checkTOFThreshold(tofMeasurements, SENSOR_TOF_RIGHT)) Serial.print("R ");
//  if (checkTOFThreshold(tofMeasurements, SENSOR_TOF_CENTER)) Serial.print("C ");
//  if (checkTOFThreshold(tofMeasurements, SENSOR_TOF_LEFT)) Serial.print("L ");

//  Serial.print("\t YAW: ");
//  Serial.print(getMedianIMUZOrientationValue(imuMeasurements));

//  Serial.print("\t Motor: ");
//  Serial.print("M_R ");
//  Serial.print(motorDirections[ACTUATOR_MOTOR_RIGHT]);
//  Serial.print("/");
//  Serial.print(motorSpeeds[ACTUATOR_MOTOR_RIGHT]);
//  Serial.print(" M_L ");
//  Serial.print(motorDirections[ACTUATOR_MOTOR_LEFT]);
//  Serial.print("/");
//  Serial.print(motorSpeeds[ACTUATOR_MOTOR_LEFT]);
//
//  Serial.print("\t Servo: ");
//  Serial.print("S_B_R ");
//  Serial.print(servoAngles[ACTUATOR_SERVO_BAR_RIGHT]);
//  Serial.print(" S_B_L ");
//  Serial.print(servoAngles[ACTUATOR_SERVO_BAR_LEFT]);
//  Serial.println("");
}
