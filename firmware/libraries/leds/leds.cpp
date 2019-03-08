/*
  leds.cpp - Led functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "config.h"

void configureLeds()
{
  pinMode(LED_RUNNING, OUTPUT);
  pinMode(LED_SYSTEM, OUTPUT);
}

// Write the let states
void writeLeds(boolean ledState[LED_COUNT])
{
  digitalWrite(LED_SYSTEM_PIN, ledState[LED_SYSTEM]);
  digitalWrite(LED_RUNNING_PIN, ledState[LED_RUNNING]);
}
