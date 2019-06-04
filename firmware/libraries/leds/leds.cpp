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
  pinMode(LED_EEPROM_PIN, OUTPUT);
  pinMode(LED_BOTTLE, OUTPUT);
  pinMode(LED_NEXT_STATE, OUTPUT);
  pinMode(LED_6, OUTPUT);
  pinMode(LED_7, OUTPUT);
}

// Write the let states
void writeLeds(boolean ledState[LED_COUNT])
{
  digitalWrite(LED_SYSTEM_PIN, ledState[LED_SYSTEM]);
  digitalWrite(LED_RUNNING_PIN, ledState[LED_RUNNING]);
  digitalWrite(LED_ALIVE_PIN, ledState[LED_ALIVE]);
  digitalWrite(LED_EEPROM_PIN, ledState[LED_EEPROM]);
  digitalWrite(LED_BOTTLE_PIN, ledState[LED_BOTTLE]);
  digitalWrite(LED_NEXT_STATE_PIN, ledState[LED_NEXT_STATE]);
  digitalWrite(LED_6_PIN, ledState[LED_6]);
  digitalWrite(LED_7_PIN, ledState[LED_7]);
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


/**
 * Generate a warn with 10Hz
 */
boolean generateWarn()
{
  return (millis() % 100 < 50);
}
