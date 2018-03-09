/**
 * The CovarianceTracker class. Analyzes X-dimensional values (from a sensor, 
 * perhaps) and calculates the covariance matrix of an input set of data.
 * 
 * @author Joshua Petrin
 * @author joshua.m.petrin@vanderbilt.edu
 * @author Vanderbilt Robotics
 * @since March 2018
 * @version 0.0?
 * @brief Calculate the covariance of a set of X-dimensional values.
 * @extends Eigen::Matrix<_Scalar, _Rows, _Cols>
 */

#ifndef COVARIANCETRACKER_CPP
#define COVARIANCETRACKER_CPP


/**
 * Default constructor. The covariance values are set to 0. 
 * Data length defaults to 100.
 */
template <typename _Scalar, int _Dimension>
CovarianceTracker<_Scalar, _Dimension>::CovarianceTracker(int len)
  : covariance(_Dimension, _Dimension), data_length(len), 
    data(_Dimension, data_length), data_covered(0)
{
}




#endif // COVARIANCETRACKER_CPP
