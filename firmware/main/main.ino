#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "EEPROM.h"
#include "config.h"
#include "utils.h"
#include "sensors.h"
#include "leds.h"
#include "actuators.h"
#include "logger.h"
#include "state-estimator.h"
#include "state-machine.h"
#include "state-following.h"
#include "state-following-slope.h" 

#include "state-following-collect.h"
#include "state-slope-down.h"
#include "state-wander.h"
#include "state-swallowing.h"
#include "state-scanning.h"
#include "state-turning.h"
#include "state-returning.h"
#include "state-emptying.h"
#include "state-poi.h"



/*********
 * Global values
 */

// Mode of the robot
int mode = m_random_navigation;
//int mode = m_poi_navigation;
//int mode = m_platform;
//int mode = m_collect;
//int mode = m_test;

// State for the state machine
int state = s_initialization;
long stateChangeTimestamp = millis();

// Global pause and weak pause
boolean globalPause = false;
boolean nGlobalPauseButtonState = true;
boolean weakPause = false;

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
  Serial.print("Mode: ");
  Serial.println(mode);
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
  configureProximity(SENSOR_PROXIMITY_DETECT, SENSOR_PROXIMITY_DETECT_PIN);
  configureProximity(SENSOR_PROXIMITY_DETECT2, SENSOR_PROXIMITY_DETECT2_PIN);

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
  setServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  setServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);

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

      stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
  }
  // Return if globally paused
  if (globalPause) return;

  // Weak pause
  if (btnState[BTN_STATE])
  {
    weakPause = true;
  }
  else if (btnState[BTN_START])
  {
    weakPause = false;
  }

  // Wait for period
  static long lastPeriodStart = millis();
  static long dt = 0;
  while (millis() - lastPeriodStart < PERIOD) { }
  dt = millis() - lastPeriodStart;
  lastPeriodStart = millis();



  #ifdef DEBUG_ENABLE
  // Testing
  if (Serial.available() > 0 && mode != m_test)
  {
    // read the incoming byte:
    char b = Serial.read();

    if (b == ' ')
    {
      stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);

      globalPause = !globalPause;
    }
    else if (b == '0') btnState[0] = !btnState[0];
    else if (b == '1') btnState[1] = !btnState[1];
    else if (b == '2') btnState[2] = !btnState[2];
    else if (b == '3') btnState[3] = !btnState[3];
    else if (b == '4') btnState[4] = !btnState[4];
    else if (b == '5') btnState[5] = !btnState[5];
    // Virtual
    else if (b == '6') btnState[6] = !btnState[6];
    else if (b == '7') btnState[7] = !btnState[7];
    else if (b == '8') btnState[8] = !btnState[8];
  }
  #endif


  // Check for mode selection (only possible in state s_initialization, s_idle or s_calibration)
  if ((state == s_initialization || state == s_idle || state == s_calibration) && btnState[BTN_MODE])
  {
    if (++mode >= MODE_LENGTH) 
    {
      mode = 0;
    }

    #ifdef SERIAL_ENABLE
    Serial.print("Mode Selection: ");
    Serial.print(mode);
    Serial.println("");
    #endif

    for (int i = 0; i<LED_COUNT; i++) 
    {
      ledState[i] = 0;
    }
    writeLeds(ledState);
    delay(500);

    ledState[mode] = 1;
    writeLeds(ledState);
    delay(1000);

    for (int i = 0; i<LED_COUNT; i++) 
    {
      ledState[i] = 0;
    }
    writeLeds(ledState);
    delay(500);
  }




  // Check for state transition
  State nextState = checkStateTransition(state, mode, stateChangeTimestamp, btnState, ledState, flags, &estimator);
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
      case s_following_slope: stateFollowingSlopeExit(); break;
      case s_following_collect: stateFollowingCollectExit(); break;
      case s_slope_down: stateSlopeDownExit(); break;
      case s_swallowing: stateSwallowingExit(); break;
      case s_scanning: stateScanningExit(); break;
      case s_turning: stateTurningExit(); break;
      case s_returning: stateReturningExit(); break;
      case s_emptying: stateEmptyingExit(); break;
      case s_poi: statePOIExit(); break;
    }

    // Enter next state
    switch (nextState)
    {
      case s_initialization: stateInitializationEnter(); break;
      case s_calibration: stateCalibrationEnter(); break;
      case s_idle: stateIdleEnter(); break;
      case s_wander: stateWanderEnter(); break;
      case s_test: stateTestEnter(); break;
      case s_following: stateFollowingEnter(); break;
      case s_following_slope: stateFollowingSlopeEnter(); break;
      case s_following_collect: stateFollowingCollectEnter(); break;
      case s_slope_down: stateSlopeDownEnter(); break;
      case s_swallowing: stateSwallowingEnter(); break;
      case s_scanning: stateScanningEnter(); break;
      case s_turning: stateTurningEnter(); break;
      case s_returning: stateReturningEnter(); break;
      case s_emptying: stateEmptyingEnter(); break;
      case s_poi: statePOIEnter(); break;
    }

    // Set next state
    state = nextState;
    stateChangeTimestamp = millis();
  }

  // Feedback
  writeLeds(ledState);

  // Update all
  readAll();

  // Update Estimator
  updateEstimator(dt);

  if (!weakPause)
  {
    switch (state)
    {
      case s_initialization: stateInitialization(); break;
      case s_calibration: stateCalibration(); break;
      case s_idle: stateIdle(); break;
      case s_wander: stateWander(); break;
      case s_test: stateTest(); break;
      case s_following: stateFollowing(); break;
      case s_following_slope: stateFollowingSlope(); break;
      case s_following_collect: stateFollowingCollect(); break;
      case s_slope_down: stateSlopeDown(); break;
      case s_swallowing: stateSwallowing(); break;
      case s_scanning: stateScanning(); break;
      case s_turning: stateTurning(); break;
      case s_returning: stateReturning(); break;
      case s_emptying: stateEmptying(); break;
      case s_poi: statePOI(); break;
    }    
  }



  // Feedback
  ledState[LED_ALIVE] = generatePing();
  ledState[LED_BOTTLE] = flags[FLAG_SWALLOWED_BOTTLE];
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
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT, SENSOR_PROXIMITY_DETECT_PIN);
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT2, SENSOR_PROXIMITY_DETECT2_PIN);
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
  bool calibrationSuccessful = false;
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  for (int i = 0; i<ESTIMATOR_CALIBRATION_MAX; i++) // Wait for IMU to get stabelized
  {
    delay(100);
    ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
    delay(100);
    ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
    readIMU(imuMeasurements, &imuMeasurementIndex);
    float newCalibrateAngle = imuMeasurements[SENSOR_IMU_YAW][imuMeasurementIndex];
    if (fabsf(newCalibrateAngle) < 1) 
    {
      #ifdef SERIAL_ENABLE
      Serial.print("?");
      #endif
      continue;
    }

    #ifdef SERIAL_ENABLE
    Serial.print("-");
    #endif
    if (fabsf(newCalibrateAngle - calibrateAngle) > 0.02)
    {
      calibrateAngle = newCalibrateAngle;
      calibrateCounter = 0;
    }
    else
    {
      #ifdef SERIAL_ENABLE
      Serial.print("o");
      if (calibrateCounter == 0)
      {
        Serial.print("(");
         Serial.print(newCalibrateAngle);
        Serial.print(")");
      }
      #endif
      calibrateCounter++;
    }

    if (calibrateCounter > ESTIMATOR_CALIBRATION_COUNTER)
    {
      #ifdef SERIAL_ENABLE
      Serial.println("!");
      #endif
      calibrationSuccessful = true;
      break;
    }
  }
  ledState[LED_SYSTEM] = !ledState[LED_SYSTEM]; writeLeds(ledState);
  #ifdef SERIAL_ENABLE
  if (!calibrationSuccessful) Serial.println("#\nFAILED!");
  #endif
  delay(200);
  setEstimatorAngleOffset();

  ledState[LED_SYSTEM] = HIGH;
  writeLeds(ledState);
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


// !!! TO USE THE TEST STATE, DISABLE THE SERIAL READ IN THE LOOP FUNCTION !!!


// The "s_test" state
void stateTestEnter()
{
  ledState[LED_RUNNING] = HIGH;
}

void stateTest()
{
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

  int p = getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD);
  int tl = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT);
  int tc = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER);
  int tr = getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT);
  if (p > 300 && (tl == 0 || tl > 32) && (tc == 0 || tc > 32) && (tr == 0 || tr > 32))
  {
    Serial.println("Bottle");
  }
  else if (p > 200 && (tl > 0 && tl < 32 || tc > 0 && tc < 32 || tr > 0 && tr < 32))
  {
    Serial.println("Wall");
  }
  else
  {
    Serial.println("–");
  }



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
//
//Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT]);
//Serial.print(", ");
//Serial.println(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT]);
//Serial.print(", ");
//Serial.println(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT]);
}

void stateTestExit()
{
  ledState[LED_RUNNING] = LOW;
  stopMotor(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
  stopMotor(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
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
  stateWanderRoutine( proximityMeasurements,
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
// ===                     FOLLOWING SLOPE STATE                ===
// ================================================================


// The "s_following_slope" state
void stateFollowingSlopeEnter()
{
  stateFollowingSlopeEnterRoutine(ledState, flags);
}

void stateFollowingSlope()
{
  stateFollowingSlopeRoutine( proximityMeasurements,
                              proximityAmbientMeasurements,
                              proximityAmbientVarianceMeasurements,
                              tofMeasurements,
                              estimator.getAngle(),
                              imuMeasurements,
                              motorSpeeds,
                              motorSpeedMeasurements,
                              servoAngles,
                              btnState,
                              ledState,
                              flags);

  updateAll();
}

void stateFollowingSlopeExit()
{
  stateFollowingSlopeExitRoutine(ledState, flags);
}

// ================================================================
// ===                    FOLLOWING COLLECT STATE               ===
// ================================================================


// The "s_following_collect" state
void stateFollowingCollectEnter()
{
  stateFollowingCollectEnterRoutine(ledState, flags);
}

void stateFollowingCollect()
{
  stateFollowingCollectRoutine( proximityMeasurements,
                                proximityAmbientMeasurements,
                                proximityAmbientVarianceMeasurements,
                                tofMeasurements,
                                estimator.getAngle(),
                                imuMeasurements,
                                motorSpeeds,
                                motorSpeedMeasurements,
                                servoAngles,
                                btnState,
                                ledState,
                                flags);

  updateAll();
}

void stateFollowingCollectExit()
{
  stateFollowingCollectExitRoutine(ledState, flags);
}


// ================================================================
// ===                       SLOPE DOWN STATE                   ===
// ================================================================


// The "s_slope_down" state
void stateSlopeDownEnter()
{
  stateSlopeDownEnterRoutine(ledState, flags);
}

void stateSlopeDown()
{
  stateSlopeDownRoutine( proximityMeasurements,
                              proximityAmbientMeasurements,
                              proximityAmbientVarianceMeasurements,
                              tofMeasurements,
                              estimator.getAngle(),
                              imuMeasurements,
                              motorSpeeds,
                              motorSpeedMeasurements,
                              servoAngles,
                              btnState,
                              ledState,
                              flags);

  updateAll();
}

void stateSlopeDownExit()
{
  stateSlopeDownExitRoutine(ledState, flags);
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
   stateSwallowingRoutine( proximityMeasurements,
                          proximityAmbientMeasurements,
                          proximityAmbientVarianceMeasurements,
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


// The "s_turning" state
void stateTurningEnter()
{
  stateTurningEnterRoutine(ledState, flags);
}

void stateTurning()
{
  stateTurningRoutine(imuMeasurements,
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


// ================================================================
// ===                      RETURNING STATE                     ===
// ================================================================


// The "s_returning" state
void stateReturningEnter()
{
  stateReturningEnterRoutine(ledState, flags);
}

void stateReturning()
{
  stateReturningRoutine(proximityMeasurements,
                        proximityAmbientMeasurements,
                        proximityAmbientVarianceMeasurements,
                        tofMeasurements,
                        estimator.getAngle(),
                        motorSpeeds,
                        motorSpeedMeasurements,
                        btnState,
                        ledState,
                        flags);

  updateAll();
}

void stateReturningExit()
{
  stateReturningExitRoutine(ledState, flags);
}



// ================================================================
// ===                        EMPTYING STATE                    ===
// ================================================================


// The "s_emptying" state
void stateEmptyingEnter()
{
  stateEmptyingEnterRoutine(ledState, flags);
}

void stateEmptying()
{
   stateEmptyingRoutine(motorSpeeds,
                        motorSpeedMeasurements,
                        servoAngles,
                        btnState,
                        ledState,
                        flags);

  updateAll();
}

void stateEmptyingExit()
{
  stateEmptyingExitRoutine(ledState, flags);
}



// ================================================================
// ===                      RETURNING STATE                     ===
// ================================================================


// The "s_poi" state
void statePOIEnter()
{
  statePOIEnterRoutine(ledState, flags);
}

void statePOI()
{
  statePOIRoutine(proximityMeasurements,
                  proximityAmbientMeasurements,
                  proximityAmbientVarianceMeasurements,
                  tofMeasurements,
                  estimator.getAngle(),
                  estimator.getPosition(),
                  motorSpeeds,
                  motorSpeedMeasurements,
                  btnState,
                  ledState,
                  flags);

  updateAll();
}

void statePOIExit()
{
  statePOIExitRoutine(ledState, flags);
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
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_DETECT, SENSOR_PROXIMITY_DETECT_PIN);
  readProximity(proximityMeasurements, proximityMeasurementIndex, SENSOR_PROXIMITY_DETECT2, SENSOR_PROXIMITY_DETECT2_PIN);

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
  float angle = imuMeasurements[SENSOR_IMU_YAW][imuMeasurementIndex] - estimatorAngleOffset;
  float speed = (motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT] + motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]) / 2.f;

  speed *= ACUTATOR_MOTOR_DISTANCE_CORRECTION;

  estimator.step(speed, -angle, dt / 1000.f);

  Matrix<2> p = estimator.getPosition();
  float a = estimator.getAngle();

//  Serial.print(p(0));
//  Serial.print(", ");
//  Serial.print(p(1));
//  Serial.print(", ");
//  Serial.print(a);
//  Serial.print(", ");
//  Serial.println(state);
}



















/**
 * Logging
 */
void log()
{
  // RESET EEPROM
  if (btnState[VIRTUAL_BTN_EEPROM_RESET])
  {
   Serial.print("RESETTING EEPROM …");
   resetLog();
   Serial.println(" DONE!");
  }

  // READ EEPROM
  if (btnState[VIRTUAL_BTN_EEPROM_READ])
  {
    Serial.println("READ EEPROM");
    Serial.println("------------------------------------------------------------------------------------------");
    Serial.println();
    int i = 0;
    while (i++ < EEPROM.length())
    {
      Serial.print((uint8_t)readLog());
      Serial.print(i % 3 == 0 ? "\n" : ",");
    }
    Serial.println();
    Serial.println("------------------------------------------------------------------------------------------");
  }


  // ENABLE / DISABLE EEPROM LOGGING
  static boolean prevBtnState = false;
  if (btnState[BTN_EEPROM] && !prevBtnState)
  {
    if (!flags[FLAG_EEPROM])
    {
      flags[FLAG_EEPROM] = true;
      Serial.println("START LOGGING TO EEPROM");
    }
    else
    {
      Serial.println("STOP LOGGING TO EEPROM");
      flags[FLAG_EEPROM] = false;
      ledState[LED_EEPROM] = false;
    }
  }
  prevBtnState = btnState[BTN_EEPROM];





  // EEPROM Logging
  // Logging position and state
  if (flags[FLAG_EEPROM])
  {
    static long eepromTimestamp = millis();
    static boolean space = true;

//    Matrix<2> p = estimator.getPosition();
//    Serial.print(p(0));
//    Serial.print(",");
//    Serial.print(p(1));
//    Serial.print(",");
//    Serial.println(state);

    if (millis() - eepromTimestamp > 1000)
    {
      eepromTimestamp = millis();

      Matrix<2> p = estimator.getPosition();
      space = writeLog((uint8_t)(p(0) * ESTIMATOR_EEPROM_SCALING));
      space = writeLog((uint8_t)(p(1) * ESTIMATOR_EEPROM_SCALING));
      space = writeLog((uint8_t)(state));
    }

    if (!space)
    {
      Serial.println("EEPROM FULL!");
      ledState[LED_EEPROM] = generateWarn();
    }
    else
    {
      ledState[LED_EEPROM] = true;
    }
  }






  // Console logging
  // Logging all possible values
  if (btnState[VIRTUAL_BTN_LOG])
  {
    static long serialTimestamp = millis();
    if (millis() - serialTimestamp > 400)
    {
      serialTimestamp = millis();

      // Timestamp
      Serial.print("TIME=");
      Serial.print(millis());
      // Mode & State
      Serial.print(" MODE=");
      Serial.print(0);
      Serial.print(" STATE=");
      Serial.print(state);
      // Flags, Buttons & LEDs
      Serial.print(" FLAGS=");
      for (int i = 0; i<FLAG_COUNT;i++) Serial.print(flags[i]);
      Serial.print(" BTNS=");
      for (int i = 0; i<BTN_COUNT;i++) Serial.print(btnState[i]);
      Serial.print(" LEDS=");
      for (int i = 0; i<LED_COUNT;i++) Serial.print(ledState[i]);

      // Estimator
      Matrix<3> e = estimator.getState();
      Serial.print(" ESTIMATOR=");
      Serial.print(e(0));
      Serial.print(", ");
      Serial.print(e(1));
      Serial.print(", ");
      Serial.print(e(2));

      // Proximity
      Serial.print(" PROX=");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_LEFT]);
      Serial.print(", ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_LEFT]);
      Serial.print(", ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD]);
      Serial.print(", ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_FORWARD_RIGHT]);
      Serial.print(", ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_RIGHT]);
      Serial.print(" | ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_BACKWARD) - proximityAmbientMeasurements[SENSOR_PROXIMITY_BACKWARD]);
      Serial.print(" | ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT2) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT]);
      Serial.print(", ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DETECT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DETECT2]);
      Serial.print(" | ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_LEFT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_RIGHT]);
      Serial.print(", ");
      Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT) - proximityAmbientMeasurements[SENSOR_PROXIMITY_DOWN_LEFT]);
      // TOF
      Serial.print(" TOF=");
      Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_LEFT));
      Serial.print(", ");
      Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER));
      Serial.print(", ");
      Serial.print(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_RIGHT));
      // IMU
      Serial.print(" IMU=");
      Serial.print(getMedianIMUZOrientationValue(imuMeasurements));
      // Motor
      Serial.print(" MOTOR=");
      Serial.print(motorSpeeds[ACTUATOR_MOTOR_LEFT]);
      Serial.print("/");
      Serial.print(motorSpeedMeasurements[ACTUATOR_MOTOR_LEFT]);
      Serial.print(",");
      Serial.print(motorSpeeds[ACTUATOR_MOTOR_RIGHT]);
      Serial.print("/");
      Serial.print(motorSpeedMeasurements[ACTUATOR_MOTOR_RIGHT]);
      // Servo
      Serial.print(" SERVO=");
      Serial.print(servoAngles[ACTUATOR_SERVO_BAR_LEFT]);
      Serial.print(",");
      Serial.println(servoAngles[ACTUATOR_SERVO_BAR_RIGHT]);


    }
  }
}
