/**
 * The CovarianceTracker class. Analyzes X-dimensional values (from a sensor, 
 * perhaps) and calculates the covariance matrix of an input set of data.
 * Wraps two instances of Eigen::Matrix.
 * 
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
#include <cstdarg>


template <typename _Scalar, int _Dimension>
class CovarianceTracker
{
public:
  /**
   * Constructor. The covariance values are set to 0. Data length is set to 100.
   *
   * @param len The number of stored data in this windowed tracker. Defaults
   *            to 100.
   */
  CovarianceTracker(int len = 100);

  ~CovarianceTracker() = default;

  CovarianceTracker(const CovarianceTracker<_Scalar, _Dimension>&) = default;

  /**
   * double addData(Eigen::Matrix<_Scalar, _Dimension, 1> point)
   *
   * Adds the specified data point to this tracker. Example:
   * <pre>
   * {@code
   * CovarianceTracker<float, 3> covtrack(100);
   * Eigen::Matrix<float, 1, 3> datapoint;  // datapoint is a 3-vector
   * datapoint << 1.0, 2.0, 3.0;  // datapoint is now <1.0, 2.0, 3.0>
   * covtrack.addData(datapoint);
   * }
   * </pre>
   * 
   * @param point The vector data point to add. Note this _must_ be a vector of
   *              identical length to _Dimension.
   * @return The fraction of the stored data matrix that is used. 
   */
  double addData(const Eigen::Matrix<_Scalar, _Dimension, 1> &point);

  /**
   * double addData(_Scalar a1, ...)
   *
   * Adds the specified data point to this tracker. <b>WARNING!</b> Does
   * not check for number of arguments, and does not check for size of
   * primitives!! If you pass too few, you will get undefined behavior and 
   * probably a null reference exception. If you pass too many, the last 
   * number(s) will be lost. If you pass a different type than what you 
   * specified in your template, you will get bad data. So, <i>ensure you have 
   * the correct _Dimension templated, and ensure you are passing your 
   * arguments correctly.</i>
   * @param a1... The data point to add. Must have length _Dimension and type
   *              _Scalar.
   * @return The fraction of the stored data matrix that is used.
   */
  double addData(_Scalar a1, ...);

  /**
   * int getDataLength(void)
   *
   * @return The number of data that can be stored in this tracker.
   */
  int getDataLength(void) const
  {
    return _data_length;
  }

  /**
   * Eigen::Matrix<>& getCovariance(void)
   * 
   * Get the covariance matrix. If no data has been inserted into this tracker,
   * returns a _Dimension x _Dimension matrix of zeros. 
   * @return The current calculated covariance matrix. 
   */
  Eigen::Matrix<_Scalar, _Dimension, _Dimension> getCovariance() const
  {
    return _covariance;
  }

  /**
   * int getDimension(void)
   *
   * @return The dimension of this covariance tracker.
   */
  int getDimension(void) const
  {
    return _Dimension;
  }

  /**
   * Eigen::Matrix<double, _Dimension, 1> getMean(void)
   *
   * @return The mean vector of the values stored in this covariance tracker.
   */
  Eigen::Matrix<double, _Dimension, 1> getMean(void) const
  {
    return _mean;
  }

  /* 
   * double getFractionUsed(void)
   *
   * @return The fraction of the data matrix that is used. (>= 0 and <= 1)
   */
  double getFractionUsed(void) const
  {
    return (static_cast<double>(_num_used_data) 
      / static_cast<double>(_data_length));
  }

private:
  int _newest_data;  // The pointer to the newest value inserted.
  int _num_used_data;  // The number of data used.
  Eigen::Matrix<_Scalar, _Dimension, 1> _running_sum;
  Eigen::Matrix<double, _Dimension, 1> _mean;
  const Eigen::Index _data_length;
  Eigen::Matrix<_Scalar, Eigen::Dynamic, _Dimension> _data;
  Eigen::Matrix<double, Eigen::Dynamic, _Dimension> _residuals;
  Eigen::Matrix<double, _Dimension, _Dimension> _covariance;
};


#include "covariance-tracker.cpp"

#endif //COVARIANCETRACKER_H
