/*
  logger.cpp - Logging functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "logger.h"
#include <EEPROM.h>

static int logAddress = 0;
static int readAddress = 0;

boolean writeLog(uint8_t v)
{
  if (logAddress >= EEPROM.length()) return false;

  EEPROM.update(logAddress++, v);

  return true;
}

int8_t readLog()
{
  if (readAddress >= EEPROM.length()) return 0;

  return EEPROM.read(readAddress++);

}

void resetLog()
{
  logAddress = 0;
  readAddress = 0;

  for (int i = 0 ; i < EEPROM.length() ; i++)
  {
    EEPROM.write(i, 0);
  }
}
