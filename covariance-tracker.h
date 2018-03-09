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
   * double addData(Eigen::Matrix<_Scalar, 1, _Dimension> point)
   *
   * Adds the specified data point to this tracker. Example:
   * <pre>
   * {@code
   * CovarianceTracker<float, 3> cov(100);
   * Eigen::Matrix<float, 1, 3> datapoint;  // datapoint is a 3-vector
   * datapoint << 1.0, 2.0, 3.0;  // datapoint is now <1.0, 2.0, 3.0>
   * cov.addData(datapoint);
   * }
   * </pre>
   * 
   * @param point The vector data point to add. Note this _must_ be a vector of
   *              identical length to _Dimension.
   * @return The fraction of the stored data matrix that is used. 
   */
  double addData(const Eigen::Matrix<_Scalar, 1, _Dimension> &point);

  /**
   * double addData(_Scalar a1, ...)
   *
   * Adds the specified data point to this tracker. Checks to ensure the length
   * of the data passed equals _Dimension. Throws a compiler error otherwise. 
   * @param a1... The data point to add. Must have length _Dimension.
   * @return The fraction of the stored data matrix that is used.
   */
  double addData(_Scalar a1, ...);

  /**
   * int getDataLength(void)
   *
   * @return The number of stored data in this tracker.
   */
  int getDataLength(void) const {
    return data_length;
  }

  /**
   * void setDataLength(int newlen)
   *
   * Sets the instance's data length to the specified value. Note that this will
   * create an entirely new matrix, inserting as many of the previous matrix's 
   * values as possible into the new one. If the new size is smaller than the 
   * old, then the oldest values will be destroyed. (Note this will also 
   * generate a new covariance matrix.) If the new size is larger, there will be 
   * no noticeable effect until additional data are added to the tracker; then 
   * the covariance matrix will be updated with more precise values.
   * @param newlen The new data length for this tracker.
   */
  void setDataLength(int newlen);

  /**
   * Eigen::Matrix<>& getCovariance(void)
   * 
   * Get the covariance matrix. If no data has been inserted into this tracker,
   * returns a _Dimension x _Dimension matrix of zeros. 
   * @return The current calculated covariance matrix. 
   */
  const Eigen::Matrix<_Scalar, _Dimension, _Dimension>& getCovariance() const {
    return covariance;
  }

  /**
   * int getDimension(void)
   *
   * @return The dimension of this covariance tracker.
   */
  int getDimension(void) const {
    return _Dimension;
  }

  /* 
   * double getUsed(void)
   *
   * @return The fraction of the data matrix that is used.
   */
  double getUsed(void) {
    return (static_cast<double>(data_covered) / static_cast<double>(data_length));
  }

private:
  int data_covered;
  Eigen::Index data_length;
  Eigen::Matrix<_Scalar, _Dimension, _Dimension> covariance;
  Eigen::Matrix<_Scalar, _Dimension, Eigen::Dynamic> data;
};


#include "covariance-tracker.cpp"

#endif //COVARIANCETRACKER_H
