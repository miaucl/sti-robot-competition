#include <config.h>
#include <utils.h>

/*********
 * Global values
 */

// State for the state machine
int state = s_initialize;

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

  // Configure
  configureSensors();
}

/**********
 * Start main loop
 */
void loop()
{
  // Read sensors
  readSensors();

  //getMedianTOFValue(SENSOR_TOF_RIGHT);

  //Serial.println(getAverageProximityValue(SENSOR_PROXIMITY_RIGHT));
  //Serial.println(checkProximityThreshold(SENSOR_PROXIMITY_RIGHT));


  //Serial.println(getMedianTOFValue(SENSOR_TOF_RIGHT));
  //Serial.println(checkTOFThreshold(SENSOR_TOF_RIGHT));

  log();
}

void log()
{
  Serial.print("S: ");
  Serial.print(state);
  Serial.print("\t PROX: ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_RIGHT)) Serial.print("R ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_FORWARD_RIGHT)) Serial.print("FR ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_FORWARD)) Serial.print("F ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_FORWARD_LEFT)) Serial.print("FL ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_LEFT)) Serial.print("L ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_BACKWARD)) Serial.print("B ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_DOWN_RIGHT)) Serial.print("DR ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_DOWN_LEFT)) Serial.print("DL ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_DETECT_RIGHT)) Serial.print("DR ");
  if (checkProximityThreshold(SENSOR_PROXIMITY_DETECT_LEFT)) Serial.print("DL ");
  Serial.print("\t TOF: ");
  if (checkTOFThreshold(SENSOR_TOF_RIGHT)) Serial.print("R ");
  
  Serial.println("");
}


// Configure all sensors
void configureSensors()
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
}

// Configure proximity sensor
void configureProximity(int id, int pin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'Proximity '");
  Serial.print(id);
  Serial.print("' on pin '");
  Serial.print(pin);
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

// Configure tof sensor
void configureTOF(int id, int triggerPin, int echoPin)
{
  #ifdef SERIAL_ENABLE
  Serial.print("Calibrate Sensor 'TOF '");
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

// Configure the buttons
void configureBtns()
{
  Serial.print("Configure Buttons");
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_PAUSE, INPUT_PULLUP);
}

// Read all sensors
void readSensors()
{
  readProximity(SENSOR_PROXIMITY_RIGHT, SENSOR_PROXIMITY_RIGHT_PIN);
  readProximity(SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN);
  readProximity(SENSOR_PROXIMITY_FORWARD, SENSOR_PROXIMITY_FORWARD_PIN);
  readProximity(SENSOR_PROXIMITY_FORWARD_LEFT, SENSOR_PROXIMITY_FORWARD_LEFT_PIN);
  readProximity(SENSOR_PROXIMITY_LEFT, SENSOR_PROXIMITY_LEFT_PIN);
  readProximity(SENSOR_PROXIMITY_BACKWARD, SENSOR_PROXIMITY_BACKWARD_PIN);
  readProximity(SENSOR_PROXIMITY_DOWN_RIGHT, SENSOR_PROXIMITY_DOWN_RIGHT_PIN);
  readProximity(SENSOR_PROXIMITY_DOWN_LEFT, SENSOR_PROXIMITY_DOWN_LEFT_PIN);
  readProximity(SENSOR_PROXIMITY_DETECT_RIGHT, SENSOR_PROXIMITY_DETECT_RIGHT_PIN);
  readProximity(SENSOR_PROXIMITY_DETECT_LEFT, SENSOR_PROXIMITY_DETECT_LEFT_PIN);

  readTOF(SENSOR_TOF_RIGHT, SENSOR_TOF_RIGHT_TRIGGER_PIN, SENSOR_TOF_RIGHT_ECHO_PIN);
}

// Read proximity sensor values
void readProximity(int id, int pin)
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

// Get the average value
int getAverageProximityValue(int id)
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

// Check if the proximity value is over the threshold
boolean checkProximityThreshold(int id)
{
  int averageMeasurement = getAverageProximityValue(id);

  return (averageMeasurement > proximityAmbientMeasurements[id] + proximityAmbientVarianceMeasurements[id] + SENSOR_PROXIMITY_THRESHOLD);
}

// Read tof sensor values
void readTOF(int id, int triggerPin, int echoPin)
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
int getMedianTOFValue(int id)
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
boolean checkTOFThreshold(int id)
{
  int medianMeasurement = getMedianTOFValue(id);

  return (medianMeasurement > 0);
}

// Read the button states
void readButtons()
{
  btn[BTN_RESET] = (digitalRead(BTN_RESET) == LOW);
  btn[BTN_START] = (digitalRead(BTN_START) == LOW);
  btn[BTN_PAUSE] = (digitalRead(BTN_PAUSE) == LOW);
}
