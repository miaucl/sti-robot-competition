#include "state-estimator.h"
#include "logger.h"
#include "BasicLinearAlgebra.h"


StateEstimator estimator;
float speed = 0;
float angle = 0;
boolean enableLogging = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);      //Set Baud Rate

  Matrix<3> x0 = {0,0,0};

  estimator.init(x0);
}

void loop() {
  // Testing
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    char b = Serial.read();

    if (b == 'w') speed += 0.05;
    if (b == 's') speed -= 0.05;
    if (b == 'a') angle += 5;
    if (b == 'd') angle -= 5;
    if (b == ' ')
    {
      speed = 0.f;
      angle = 0.f;
    }
    if (b == 'l' && !enableLogging)
    {
      resetLog();
      enableLogging = true;
    }
    if (b == 'r')
    {
      enableLogging = false;

      for (int i = 0;i<2048;i++)
      {
        Serial.print(readLog());
        Serial.print(",");
        Serial.println(readLog());
      }
    }
  }

  estimator.step(speed, angle, 0.01);

  delay(10);

  Matrix<2> p = estimator.getPosition();


  // Serial
  static int ignore = 0;
  if (ignore++ % 8 == 0)
  {
    Serial.print(p(0));
    Serial.print(",");
    Serial.print(p(1));
    Serial.print(",");
    Serial.println(millis() / 5000);
  }


  // EEPROM
  static long t = millis();
  static boolean space = true;
  if (millis() - t > 1000 && space && enableLogging)
  {
    t = millis();
    space = writeLog((int8_t)(p(0) * 100));
    space = writeLog((int8_t)(p(1) * 100));
    space = writeLog((int8_t)(millis() / 5000));
  }
  else if (!space)
  {
    Serial.println("FULL");
  }
}
