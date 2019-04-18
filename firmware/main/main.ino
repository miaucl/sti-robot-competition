#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "config.h"
#include "utils.h"
#include "sensors.h"
#include "leds.h"
#include "actuators.h"
#include "state-machine.h"
#include "state-analysis.h"
#include "state-wander.h"

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
double motorPositionMeasurements[ACTUATOR_MOTOR_COUNT] = {0};

// The angle for the servo actuators
int servoAngles[ACTUATOR_SERVO_COUNT] = {0};


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
  while (millis() - lastPeriodStart < PERIOD) { }
  lastPeriodStart = millis();





  // Check for state transition
  State nextState = checkStateTransition(state, stateChangeTimestamp, btnState);
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
      case s_analysis: stateAnalysisExit(); break;
    }

    // Enter next state
    switch (nextState)
    {
      case s_initialization: stateInitializationEnter(); break;
      case s_calibration: stateCalibrationEnter(); break;
      case s_idle: stateIdleEnter(); break;
      case s_wander: stateWanderEnter(); break;
      case s_test: stateTestEnter(); break;
      case s_analysis: stateTestEnter(); break;
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
    case s_analysis: stateAnalysis(); break;
  }

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
  ledState[LED_SYSTEM] = HIGH;
}

void stateCalibration()
{
  #ifdef SERIAL_ENABLE
  Serial.println("Calibrate Sensors");
  #endif

  delay(200);

  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_RIGHT, SENSOR_PROXIMITY_RIGHT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD, SENSOR_PROXIMITY_FORWARD_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT, SENSOR_PROXIMITY_FORWARD_LEFT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_LEFT, SENSOR_PROXIMITY_LEFT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_BACKWARD, SENSOR_PROXIMITY_BACKWARD_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT, SENSOR_PROXIMITY_DOWN_RIGHT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_LEFT, SENSOR_PROXIMITY_DOWN_LEFT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT, SENSOR_PROXIMITY_DETECT_RIGHT_PIN);
  calibrateProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_LEFT, SENSOR_PROXIMITY_DETECT_LEFT_PIN);
  calibrateTOF(SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);
  calibrateTOF(SENSOR_TOF_CENTER, SENSOR_TOF_CENTER_TRIGGER_PIN, SENSOR_TOF_CENTER_ECHO_PIN);
  calibrateTOF(SENSOR_TOF_LEFT, SENSOR_TOF_LEFT_TRIGGER_PIN, SENSOR_TOF_LEFT_ECHO_PIN);
  calibrateIMU();

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
}

void stateIdle()
{
  readBtns(btnState);
}

void stateIdleExit()
{
}

// ================================================================
// ===                        TEST STATE                        ===
// ================================================================


// The "s_test" state
void stateTestEnter()
{
  ledState[LED_RUNNING] = HIGH;
}

void stateTest()
{
  readAll();

  // Testing
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    char b = Serial.read();

    if (b == 32)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] = 0;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] = 0;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    else if (b == 119)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] += 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] += 0.1;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    else if (b == 115)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] -= 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] -= 0.1;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    else if (b == 97)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] += 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] -= 0.1;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    else if (b == 100)
    {
      motorSpeeds[ACTUATOR_MOTOR_RIGHT] -= 0.1;
      motorSpeeds[ACTUATOR_MOTOR_LEFT] += 0.1;
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN);
      writeMotorSpeed(motorSpeeds, ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN);
    }
    else if (b == 111)
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_OPEN;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_OPEN;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
    }
    else if (b == 108)
    {
      servoAngles[ACTUATOR_SERVO_BAR_RIGHT] = ACTUATOR_SERVO_BAR_RIGHT_CLOSED;
      servoAngles[ACTUATOR_SERVO_BAR_LEFT] = ACTUATOR_SERVO_BAR_LEFT_CLOSED;
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
      writeServoAngle(servoAngles, ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
    }
  }


  updateAll();

  Serial.print("Z: ");
  Serial.print(getMedianIMUZOrientationValue(imuMeasurements));
  Serial.print("\tIR: ");
  Serial.print(getAverageProximityValue(proximityMeasurements, SENSOR_PROXIMITY_FORWARD));
  Serial.print("\tTOF: ");
  Serial.println(getFilteredAverageTOFValue(tofMeasurements, SENSOR_TOF_CENTER));
//
//  static double m[2] = {0};
//
//  m[0] += (motorPositionMeasurements[0] + motorPositionMeasurements[1]) / 2; // Directional speed
//  m[1] += abs(motorPositionMeasurements[0] - motorPositionMeasurements[1]) * 2; // Rotational speed
//
//  Serial.print(millis());
//  Serial.print(",");
//  Serial.print(m[0]);
//  Serial.print(",");
//  Serial.println(m[1]);
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
  stateWanderEnterRoutine(ledState);
}

void stateWander()
{
  readAll();

  stateWanderRoutine(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, tofMeasurements, imuMeasurements, motorSpeeds, motorPositionMeasurements, btnState, ledState);

  updateAll();
}

void stateWanderExit()
{
  stateWanderExitRoutine(motorSpeeds, ledState);
}

// ================================================================
// ===                        ANALYSIS STATE                    ===
// ================================================================


// The "s_analysis" state
void stateAnalysisEnter()
{
  stateAnalysisEnterRoutine(ledState);
}

void stateAnalysis()
{
  readAll();

  stateAnalysisRoutine(proximityMeasurements, tofMeasurements, imuMeasurements, motorSpeeds, motorPositionMeasurements, ledState);

  updateAll();
}

void stateAnalysisExit()
{
  stateAnalysisExitRoutine(ledState);
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
  updateMotorSpeedControl(ACTUATOR_MOTOR_RIGHT, ACTUATOR_MOTOR_RIGHT_DIRECTION_PIN, ACTUATOR_MOTOR_RIGHT_SPEED_PIN, motorPositionMeasurements);
  updateMotorSpeedControl(ACTUATOR_MOTOR_LEFT, ACTUATOR_MOTOR_LEFT_DIRECTION_PIN, ACTUATOR_MOTOR_LEFT_SPEED_PIN, motorPositionMeasurements);
  updateServoAngleControl(ACTUATOR_SERVO_BAR_RIGHT, ACTUATOR_SERVO_BAR_RIGHT_PIN);
  updateServoAngleControl(ACTUATOR_SERVO_BAR_LEFT, ACTUATOR_SERVO_BAR_LEFT_PIN);
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
