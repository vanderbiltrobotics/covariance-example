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
 * Constructor. The covariance values are set to 0. Data length defaults to 100.
 */
template <typename _Scalar, int _Dimension>
CovarianceTracker<_Scalar, _Dimension>::CovarianceTracker(int len)
  : _newest_data(-1),
    _num_used_data(0),
    _running_sum(_Dimension),
    _mean(_Dimension),
    _data_length(len),
    _data(len, _Dimension),
    _residuals(len, _Dimension),
    _covariance(_Dimension, _Dimension)
{
}

/**
 * double addData(Eigen::Matrix<_Scalar, _Dimension, 1> point)
 *
 * Adds the specified data point to this tracker. Example:
 * <pre>
 * {@code
 * CovarianceTracker<float, 3> cov(100);
 * Eigen::Matrix<float, 3, 1> datapoint;  // datapoint is a 3-vector
 * datapoint << 1.0, 2.0, 3.0;  // datapoint is now <1.0, 2.0, 3.0>
 * cov.addData(datapoint);
 * }
 * </pre>
 * 
 * @param point The vector data point to add. Note this _must_ be a vector of
 *              identical length to _Dimension.
 * @return The fraction of the stored data matrix that is used. 
 */
template <typename _Scalar, int _Dimension>
double CovarianceTracker<_Scalar, _Dimension>
::addData(const Eigen::Matrix<_Scalar, _Dimension, 1> &point)
{
  // update the newest data marker
  if (_newest_data+1 >= _data_length)
    _newest_data = 0;
  else
    ++_newest_data;

  // if we're going to overwrite data, grab it
  Eigen::Matrix<_Scalar, _Dimension, 1> olddata;
  if (_num_used_data >= _data_length) {
    for (int i = 0; i < _Dimension; ++i)
      olddata(i) = _data(_newest_data, i);
  } else {
    for (int i = 0; i < _Dimension; ++i)
      olddata(i) = 0;
    ++_num_used_data;
  }

  Eigen::Matrix<double, _Dimension, 1> mean_diff;

  for (int i = 0; i < _Dimension; ++i)
  {
    // insert new data into the matrix
    _data(_newest_data, i) = point(i);
    // calculate the new mean vector
    _running_sum(i) += point(i) - olddata(i);
    _mean(i) = static_cast<double>(_running_sum(i))
               / static_cast<double>(_num_used_data);
    // calculate the residual matrix (n^2)
    for (int j = 0; j < _num_used_data; ++j) {
      _residuals(j, i) = _data(j, i) - _mean(i);
    }
    // calculate new covariance matrix
    _covariance = _residuals.block(0, 0, _num_used_data, _Dimension).transpose()
                  * _residuals.block(0, 0, _num_used_data, _Dimension)
                  / (static_cast<double>(_num_used_data) - 1.0);
  }

  // For debugging.
  //std::cout << _data << std::endl;
  //std::cout << _residuals << std::endl;

  return getFractionUsed();
}

/**
 * double addData(_Scalar a1, ...)
 *
 * Adds the specified data point to this tracker. 
 * @param a1... The data point to add. Must have length _Dimension.
 * @return The fraction of the stored data matrix that is used.
 */
template <typename _Scalar, int _Dimension>
double CovarianceTracker<_Scalar, _Dimension>::addData(_Scalar a1, ...)
{
  va_list args;
  va_start(args, a1);
  Eigen::Matrix<_Scalar, _Dimension, 1> point; 
  point(0) = a1;
  for (int i = 1; i < _Dimension; ++i)
    point(i) = va_arg(args, _Scalar);
  va_end(args);
  return addData(point);
}

#endif // COVARIANCETRACKER_CPP
