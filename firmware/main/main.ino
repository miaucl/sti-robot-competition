#include <Thread.h>
#include <StaticThreadController.h>
#include <config.h>
#include <controller.h>
#include <sensors.h>
#include <sensor-proximity.h>


// Define the threads and the thread controller
#define THREAD_COUNT 3
StaticThreadController<THREAD_COUNT> *staticThreadController;

// Define the controller
Controller *controller;
// Define the sensors
Sensors *sensors;
// Define the proximity sensor 'test'
SensorProximity *sensorProximityTest;

// Initialization
void setup() {
#ifdef SERIAL_ENABLE
  // Setup serial communication
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.print("ROBOT ID: ");
  Serial.println(ROBOT_ID);
  Serial.println("Start Up …");
#endif

  // Create the proximity sensor 'test'
  sensorProximityTest = new SensorProximity(SENSOR_PROXIMITY_TEST, SENSOR_PROXIMITY_TEST_PIN);

  // Create the sensors object
  sensors = new Sensors({sensorProximityTest}, 1);

  // Create the controller object
  controller = new Controller();

  // Create the thread controller
  staticThreadController = new StaticThreadController<THREAD_COUNT>(controller, sensors, sensorProximityTest);

  // Calibrate the sensors of the robot
  sensors->calibrate();

  // Start the robot
  controller->start();
}

// Start main loop
void loop()
{
  // Start the threads
  staticThreadController->run();
}
