/*
  leds.h - Led functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef Led_h
#define Led_h

/**
 * Configure the leds
 */
void configureLeds();

/**
 * Write the led states
 */
void writeLeds(boolean ledState[LED_COUNT]);

#endif
