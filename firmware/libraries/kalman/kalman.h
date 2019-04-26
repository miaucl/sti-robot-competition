/*
  kalman.h - Kalman functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef KALMAN_h
#define KALMAN_h

#define KF_X 0
#define KF_Y 1
#define KF_P 2
#define KF_V 3
#define KF_O 4

#include "BasicLinearAlgebra.h"


using namespace BLA;


class KalmanFilter {
  public:
  	KalmanFilter() {};

    void init(Matrix<5> w, Matrix<5,5> h, Matrix<5> x0, Matrix<5,5> p0);

    void predict();

  private:
    Matrix<5> _x;
    Matrix<5,5> _p;

    Matrix<5> _w;
    Matrix<5,5> _ggt;
    Matrix<5,5> _h;

    Matrix<5,5> KalmanFilter::calculateSystemNoise(Matrix<5> w);
    Matrix<5,5> KalmanFilter::getMatrixPHI(float v, float p);
    // Matrix<10,10> KalmanFilter::calculateAuxiliaryMatrixB(Matrix<10,10> a);
};

#endif
