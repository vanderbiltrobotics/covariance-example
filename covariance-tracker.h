/**
 * The CovarianceTracker class. Analyzes X-dimensional values (from a sensor, 
 * perhaps) and calculates the covariance matrix of an input set of data.
 * Wraps two instances of Eigen::Matrix.
 * 
 * @author Joshua Petrin
 * @author joshua.m.petrin@vanderbilt.edu
 * @author Vanderbilt Robotics
 * @since March 2018
 * @version 0.0?
 * @brief Calculate the covariance of a set of X-dimensional values.
 */

#ifndef COVARIANCETRACKER_H
#define COVARIANCETRACKER_H

#include <Eigen/Dense>


template <typename _Scalar, int _Dimension>
class CovarianceTracker
{
public:
  
  Eigen::Matrix<_Scalar, _Dimension, _Dimension> covariance;

  /**
   * Default constructor. The covariance values are set to 0. 
   * Data length defaults to 100.
   */
  CovarianceTracker();

  /**
   * 
   */

private:
  int data_covered;
  Eigen::Index datalength;
  Eigen::Matrix<_Scalar, _Dimension, Eigen::Dynamic> data;
};


#include "covariance-tracker.cpp"

#endif //COVARIANCETRACKER_H
