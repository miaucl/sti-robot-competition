#include "kalman.h"
#import "BasicLinearAlgebra.h"


KalmanFilter kalman;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);      //Set Baud Rate

  Matrix<5> w = {0.1,0.1,0.05,0.2,0.4};
  Matrix<5,5> h;
  h.Fill(0);

  Matrix<5> x0 = {0,0,15,0.1,0};
  Matrix<5,5> p0;
  p0.Fill(0);

  kalman.init(w,h,x0,p0);

  kalman.predict();
}

void loop() {
  // put your main code here, to run repeatedly:

}
