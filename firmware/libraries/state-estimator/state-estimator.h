/*
  state-estimator.h - State estimator functions
  Created by Cyrill Lippuner, 2019.
*/
#ifndef STATE_ESTIMATOR_h
#define STATE_ESTIMATOR_h


#include "BasicLinearAlgebra.h"


using namespace BLA;

/**
 * A state estimator for the robots position and rotation
 */
class StateEstimator
{
  public:
  	StateEstimator() {};

    /**
     * Initialize the robot with a zero state
     */
    void init(Matrix<3> x0);

    /**
     * Perform a step by providing the speed, the angle and the time step
     */
    void step(float speed, float angle, float dt);
    /**
     * Get the full state
     */
    Matrix<3> getState();
    /**
     * Get the position
     */
    Matrix<2> getPosition();
    /**
     * Get the angle
     */
    float getAngle();

  private:
    Matrix<3> _x;

};

#endif
