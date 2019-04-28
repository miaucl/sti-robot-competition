/*
  state-estimator.cpp - State estimator functions
  Created by Cyrill Lippuner, 2019.
*/

#include "Arduino.h"
#include "state-estimator.h"
#include "math.h"

#include "BasicLinearAlgebra.h"
#include "DATA.h"


using namespace BLA;


void StateEstimator::init(Matrix<3> x0)
{
  this->_x = x0;
}

void StateEstimator::step(float speed, float angle, float dt)
{
  float x = this->_x(0);
  float y = this->_x(1);
  float phi = this->_x(2);

  this->_x(0) = x + (cos(phi / 180 * M_PI) + cos(angle / 180 * M_PI)) / 2. * speed * dt; // Approximation
  this->_x(1) = y + (sin(phi / 180 * M_PI) + sin(angle / 180 * M_PI)) / 2. * speed * dt; // Approximation
  this->_x(2) = angle;
}


Matrix<3> StateEstimator::getState()
{
  Matrix<3> s;
  s(0) = this->_x(0);
  s(1) = this->_x(1);
  s(2) = this->_x(2);

  return s;
}

Matrix<2> StateEstimator::getPosition()
{
  Matrix<2> p;
  p(0) = this->_x(0);
  p(1) = this->_x(1);

  return p;
}

float StateEstimator::getAngle()
{
  return this->_x(2);
}
