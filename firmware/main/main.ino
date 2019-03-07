#include <config.h>
#include <controller.h>
#include <sensor-proximity.h>
#include <sensor-time-of-flight.h>


// Define the controller
Controller *controller;
// Define the horizontal proximity sensors
SensorProximity *horizontalProximitySensors;
// Define the time-of-flight sensors
SensorTimeOfFlight *timeOfFlightSensors;


// Initialization
void setup() {
  #ifdef SERIAL_ENABLE
  // Setup serial communication
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.print("ROBOT ID: ");
  Serial.println(ROBOT_ID);
  Serial.println("Start Up …");
  #endif

  // Create the proximity sensors
  SensorProximity sensorProximityList[SENSOR_PROXIMITY_HORIZONTAL_COUNT] = 
  {
    SensorProximity(SENSOR_PROXIMITY_RIGHT,         SENSOR_PROXIMITY_RIGHT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_FORWARD,       SENSOR_PROXIMITY_FORWARD_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_FORWARD_LEFT,  SENSOR_PROXIMITY_FORWARD_LEFT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_LEFT,          SENSOR_PROXIMITY_LEFT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_BACKWARD,      SENSOR_PROXIMITY_BACKWARD_PIN, THRESHOLD_HORIZONTAL_PROXIMITY)
  };
   
  // Pick the horizontal proximity sensors
  horizontalProximitySensors = sensorProximityList + SENSOR_PROXIMITY_HORIZONTAL_OFFSET;

  // Create the time-of-flight sensors
  SensorTimeOfFlight sensorTimeOfFlightList[1] = 
  {
    SensorTimeOfFlight(SENSOR_TIME_OF_FLIGHT_RIGHT, SENSOR_TIME_OF_FLIGHT_RIGHT_PIN_TRIGGER, SENSOR_TIME_OF_FLIGHT_RIGHT_PIN_ECHO, THRESHOLD_DISTANCE_TIME_OF_FLIGHT)
  }; 

  // Pick the time-of-flight sensors
  timeOfFlightSensors = sensorTimeOfFlightList;

  // Create the controller object
  controller = new Controller(sensorProximityList, timeOfFlightSensors);

  // Start the robot
  controller->start();

  // Horizontal proximity sensors
  for (int i = 0; i<SENSOR_PROXIMITY_HORIZONTAL_COUNT; i++)
  {
    (horizontalProximitySensors + i)->calibrate();
  }

  
  // Time-of-flight sensors
  for (int i = 0; i<SENSOR_TIME_OF_FLIGHT_COUNT; i++)
  {
    (timeOfFlightSensors + i)->calibrate();
  }
  
  // Wait at bit
  delay(500);
 }






// Start main loop
void loop()
{
  // Get current timestamp and check for each thread if it should be runned
  unsigned long now = millis();
  
  // Controller
  if (now - controller->getLastRunned() > CONTROLLER_PERIOD) 
    controller->run(now);

  // Horizontal proximity sensors
  for (int i = 0; i<SENSOR_PROXIMITY_HORIZONTAL_COUNT; i++)
  {
    if ((now - (horizontalProximitySensors + i)->getLastRunned()) > SENSOR_PROXIMITY_HORIZONAL_PERIOD) 
      (horizontalProximitySensors + i)->run(now);
  }

  
  // Time-of-flight sensors
  for (int i = 0; i<SENSOR_TIME_OF_FLIGHT_COUNT; i++)
  {
    (timeOfFlightSensors + i)->loop();
    if ((now - (timeOfFlightSensors + i)->getLastRunned()) > SENSOR_TIME_OF_FLIGHT_PERIOD) 
      (timeOfFlightSensors + i)->run(now);
  }
  

  delay(1);  
}
