/*
  kalman.cpp - Kalman functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "kalman.h"
#include "math.h"

#include "BasicLinearAlgebra.h"
#include "DATA.h"


using namespace BLA;


void KalmanFilter::init(Matrix<5> w, Matrix<5,5> h, Matrix<5> x0, Matrix<5,5> p0)
{
  Serial << "w: " << w << '\n';
  Serial << "h: " << h << '\n';
  Serial << "x0: " << x0 << '\n';
  Serial << "p0: " << p0 << '\n';

  this->_w = w;
  this->_ggt = this->calculateSystemNoise(this->_w);
  this->_h = h;
  this->_x = x0;
  this->_p = p0;

  Serial << "ggt: " << this->_ggt << '\n';

}

void KalmanFilter::predict()
{
  float v = this->_x(KF_V);
  float p = this->_x(KF_P);
  if (p < 0) p += 360;

  //Matrix<5,5> phi = this->getMatrixPHI(0,0);
  this->getMatrixPHI(0.2,0);
  this->getMatrixPHI(0.2,45);
  this->getMatrixPHI(0.2,90);
  this->getMatrixPHI(0.2,145);
}

Matrix<5,5> KalmanFilter::calculateSystemNoise(Matrix<5> w)
{
  Matrix<5,5> ggt;
  ggt.Fill(0);
  ggt(0,0) = w(0) * w(0);
  ggt(1,1) = w(1) * w(1);
  ggt(2,2) = w(2) * w(2);
  ggt(3,3) = w(3) * w(3);
  ggt(4,4) = w(4) * w(4);

  return ggt;
}



Matrix<5,5> KalmanFilter::getMatrixPHI(float v, float p)
{
  int vIndex = 0;
  float vProportion = 0;
  for (int i = 0;i<V_LENGTH-1;i++)
  {
    if (V[i] < v && v <= V[i+1])
    {
      vIndex = i;
      vProportion = (v - V[i]) / (V[i+1]- V[i]);
      break;
    }
  }

  int pIndex = 0;
  float pProportion = 0;
  for (int i = 0;i<P_LENGTH-1;i++)
  {
    if (P[i] < p && p <= P[i+1])
    {
      pIndex = i;
      pProportion = (p - P[i]) / (P[i+1]- P[i]);
      break;
    }
  }

  Matrix<5,5> phi;
  phi.Fill(0);

  for (int i = 0; i<PHI_IDX_LENGTH;i++)
  {
    phi(PHI_IDX[i]/5,PHI_IDX[i]%5) = PHI[vIndex+1][pIndex+1][i];
  }


  //Serial << "v index: " << vIndex << " / " << vProportion << '\n';
  //Serial << "p index: " << pIndex << " / " << pProportion << '\n';

  Serial << "predict phi: " << phi * 100 << '\n';

  return phi;
}
//
//
// Matrix<10,10> KalmanFilter::calculateAuxiliaryMatrixB(Matrix<10,10> a)
// {
//   // Matrix<5,5> zero;
//   // zero.Fill(0);
//   // Matrix<10,10> a = ((-f || ggt) && (zero || ~f)) * dt;
//   // return a;
// }
