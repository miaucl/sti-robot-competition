/*
  leds.cpp - Led functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"

void configureLeds()
{
  pinMode(LED_RUNNING_PIN, OUTPUT);
  pinMode(LED_SYSTEM_PIN, OUTPUT);
  pinMode(LED_ALIVE_PIN, OUTPUT);
}

// Write the let states
void writeLeds(boolean ledState[LED_COUNT])
{
  digitalWrite(LED_SYSTEM_PIN, ledState[LED_SYSTEM]);
  digitalWrite(LED_RUNNING_PIN, ledState[LED_RUNNING]);
  digitalWrite(LED_ALIVE_PIN, ledState[LED_ALIVE]);
}


/**
 * Generate a blink with 1Hz
 */
boolean generateBlink()
{
  return (millis() % 1000 < 500);
}

/**
 * Generate a ping with 0.5Hz
 */
boolean generatePing()
{
  return (millis() % 3000 < 50);
}
