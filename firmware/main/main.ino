#include <Thread.h>
#include <StaticThreadController.h>
#include <config.h>
#include <controller.h>
#include <sensor.h>
#include <sensors.h>
#include <sensor-proximity.h>


// Define the threads and the thread controller
#define THREAD_COUNT 8
StaticThreadController<THREAD_COUNT> *staticThreadController;

// Define the controller
Controller *controller;
// Define the sensors
Sensors *sensors;


// Initialization
void setup() {
  #ifdef SERIAL_ENABLE
  // Setup serial communication
  Serial.begin(SERIAL_BAUD_RATE);

  Serial.print("ROBOT ID: ");
  Serial.println(ROBOT_ID);
  Serial.println("Start Up …");
  #endif

  // Create the horizontal proximity sensors
  SensorProximity sensorProximityList[6] = 
  {
    SensorProximity(SENSOR_PROXIMITY_RIGHT,         SENSOR_PROXIMITY_RIGHT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_FORWARD_RIGHT, SENSOR_PROXIMITY_FORWARD_RIGHT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_FORWARD,       SENSOR_PROXIMITY_FORWARD_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_FORWARD_LEFT,  SENSOR_PROXIMITY_FORWARD_LEFT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_LEFT,          SENSOR_PROXIMITY_LEFT_PIN, THRESHOLD_HORIZONTAL_PROXIMITY),
    SensorProximity(SENSOR_PROXIMITY_BACKWARD,      SENSOR_PROXIMITY_BACKWARD_PIN, THRESHOLD_HORIZONTAL_PROXIMITY)
  }; 

  // Create the sensors object
  sensors = new Sensors(sensorProximityList, 6);

  // Create the controller object
  controller = new Controller(sensors);

  // Create the thread controller
  staticThreadController = new StaticThreadController<THREAD_COUNT>(controller, 
                                                                    sensors,
                                                                    &sensorProximityList[0],
                                                                    &sensorProximityList[1],
                                                                    &sensorProximityList[2],
                                                                    &sensorProximityList[3],
                                                                    &sensorProximityList[4],
                                                                    &sensorProximityList[5]
                                                                    );

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
