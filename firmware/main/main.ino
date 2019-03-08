#include <config.h>
#include <utils.h>
#include <state-machine.h>
#include <sensors.h>
#include <leds.h>

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

// The buttons for control
boolean btnState[BTN_COUNT] = {0};

// The leds for feedback
boolean ledState[LED_COUNT] = {0};


/*********
 * Initialization
 */
void setup() {
  #ifdef SERIAL_ENABLE
  // Setup serial communication
  Serial.begin(SERIAL_BAUD_RATE);

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

  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_RIGHT, SENSOR_PROXIMITY_RIGHT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD, SENSOR_PROXIMITY_FORWARD_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT, SENSOR_PROXIMITY_FORWARD_LEFT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_LEFT, SENSOR_PROXIMITY_LEFT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_BACKWARD, SENSOR_PROXIMITY_BACKWARD_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT, SENSOR_PROXIMITY_DOWN_RIGHT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_LEFT, SENSOR_PROXIMITY_DOWN_LEFT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT, SENSOR_PROXIMITY_DETECT_RIGHT_PIN);
  configureProximity(proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_LEFT, SENSOR_PROXIMITY_DETECT_LEFT_PIN);
  
  configureTOF(SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);

  configureBtns();

  configureLeds();
  digitalWrite(11, HIGH);
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
  




  // Check for state transition
  State nextState = checkStateTransition(state, stateChangeTimestamp, btnState);
  if (state != nextState)
  {
    Serial.print("State Transition: ");
    Serial.print(state);
    Serial.print(" > ");
    Serial.print(nextState);
    Serial.println("");


    // Exit previous state
    switch (state)
    {
      case s_initialization: stateInitializationExitRoutine(); break;
      case s_calibration: stateCalibrationExitRoutine(); break;
      case s_idle: stateIdleExitRoutine(); break;  
      case s_test: stateTestExitRoutine(); break;  
    }
      
    // Enter next state
    switch (nextState)
    {
      case s_initialization: stateInitializationEnterRoutine(); break;
      case s_calibration: stateCalibrationEnterRoutine(); break;
      case s_idle: stateIdleEnterRoutine(); break;  
      case s_test: stateTestEnterRoutine(); break;  
    }

    // Set next state
    state = nextState;
    stateChangeTimestamp = millis();
  }

  // Feedback
  writeLeds(ledState);

  switch (state)
  {
    case s_initialization: stateInitializationRoutine(); break;
    case s_calibration: stateCalibrationRoutine(); break;
    case s_idle: stateIdleRoutine(); break;  
    case s_test: stateTestRoutine(); break;  
  }

  // Feedback
  writeLeds(ledState);
  

  // Logging
  log();
}

// The "s_initialization" state routines
void stateInitializationEnterRoutine()
{
  // Do nothing
}

void stateInitializationRoutine()
{
  // Do nothing
}

void stateInitializationExitRoutine()
{
  // Do nothing
}


// The "s_calibration" state routine
void stateCalibrationEnterRoutine()
{
  ledState[LED_SYSTEM] = HIGH;
}

void stateCalibrationRoutine()
{
  #ifdef SERIAL_ENABLE
  Serial.println("Calibrete Sensors");
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

  readBtns(btnState);
}

void stateCalibrationExitRoutine()
{
  ledState[LED_SYSTEM] = LOW;
}

// The "s_idle" state routine
void stateIdleEnterRoutine()
{
}

void stateIdleRoutine()
{
  readBtns(btnState);
}

void stateIdleExitRoutine()
{
}

// The "s_test" state routine
void stateTestEnterRoutine()
{
  ledState[LED_RUNNING] = HIGH;
}

void stateTestRoutine()
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

  readTOF(tofMeasurements, tofMeasurementIndex, SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);
}

void stateTestExitRoutine()
{
  ledState[LED_RUNNING] = LOW;
}

/**
 * Logging
 */
void log()
{
  Serial.print("S: ");
  Serial.print(state);
  Serial.print("\t PROX: ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_RIGHT)) Serial.print("R ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_RIGHT)) Serial.print("FR ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD)) Serial.print("F ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_FORWARD_LEFT)) Serial.print("FL ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_LEFT)) Serial.print("L ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_BACKWARD)) Serial.print("B ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_RIGHT)) Serial.print("DR ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DOWN_LEFT)) Serial.print("DL ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_RIGHT)) Serial.print("DR ");
  if (checkProximityThreshold(proximityMeasurements, proximityAmbientMeasurements, proximityAmbientVarianceMeasurements, SENSOR_PROXIMITY_DETECT_LEFT)) Serial.print("DL ");
  Serial.print("\t TOF: ");
  if (checkTOFThreshold(tofMeasurements, SENSOR_TOF_RIGHT)) Serial.print("R ");

  Serial.println("");
}
