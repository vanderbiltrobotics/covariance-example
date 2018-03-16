/**
 * The CovarianceTracker class. Analyzes X-dimensional values (from a sensor, 
 * perhaps) and calculates the covariance matrix of an input set of data.
 * 
 * @author Joshua Petrin
 * @author joshua.m.petrin@vanderbilt.edu
 * @author Vanderbilt Robotics
 * @since March 2018
 * @version 0.1?
 * @brief Calculate the covariance of a set of X-dimensional values.
 */

#ifndef COVARIANCETRACKER_CPP
#define COVARIANCETRACKER_CPP

//#if __cplusplus <= 199711L
//  #error This library needs at least C++11! Compile with -std=c++11 or gnu++11.
//#endif


/**
 * Constructor. The covariance values are set to 0. Data length defaults to 100.
 */
template <typename _Scalar, int _Dimension>
CovarianceTracker<_Scalar, _Dimension>::CovarianceTracker(int len)
  : _newest_data(-1),
    _num_used_data(0),
    _running_sum(),
    _mean(),
    _data_length(len),
    _data(len, _Dimension),
    _residuals(len, _Dimension),
    _covariance()
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
    // calculate the residual matrix (n^2 algorithm...)
    for (int j = 0; j < _num_used_data; ++j) {
      _residuals(j, i) = static_cast<double>(_data(j, i)) - _mean(i);
    }
    // calculate new covariance matrix
    _covariance = _residuals.block(0, 0, _num_used_data, _Dimension).transpose()
                  * _residuals.block(0, 0, _num_used_data, _Dimension).eval()
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
 * Do NOT try to use this function. It introduces several problems in the 
 * way arguments are passed via the stack. Odds are, if you use it, you will 
 * use it incorrectly, your covariance will be garbage, and you will come 
 * complaining to me about a problem that has already become obsolete. 
 * 
 * <strike>Adds the specified data point to this tracker. <b>WARNING!</b> Does
 * not check for number of arguments, and does not check for size of
 * primitives!! If you pass too few, you will get undefined behavior and 
 * probably a null reference exception. If you pass too many, the last 
 * number(s) will be lost. If you pass a different type than what you 
 * specified in your template, you will get bad data. So, <i>ensure you have 
 * the correct _Dimension templated, and ensure you are passing your 
 * arguments correctly.</i></strike>
 * @param a1... The data point to add. Must have length _Dimension and type
 *              _Scalar.
 * @return The fraction of the stored data matrix that is used.
 * @depricated 0.1
 */
// template <typename _Scalar, int _Dimension>
// double CovarianceTracker<_Scalar, _Dimension>::addData(_Scalar a1, ...)
// {
//   va_list args;
//   va_start(args, a1);
//   Eigen::Matrix<_Scalar, _Dimension, 1> point; 
//   point(0) = a1;
//   for (int i = 1; i < _Dimension; ++i)
//     point(i) = va_arg(args, _Scalar);
//   va_end(args);
//   return addData(point);
// }

/**
 * double addData(const std::vector<_Scalar> point)
 *
 * Adds the specified data point to this tracker. Asserts the size of point
 * is equal to _Dimension! So only pass the correct number of arguments or
 * you will get a runtime error. 
 * @param point The std::vector<_Scalar> containing the data
 * @return The fraction of the stored data matrix that is used.
 */
template <typename _Scalar, int _Dimension>
double CovarianceTracker<_Scalar, _Dimension>
::addData(const std::vector<_Scalar> point)
{
  // point.size() MUST be equal to the Dimension of this 
  assert(point.size() == _Dimension);
  Eigen::Matrix<_Scalar, _Dimension, 1> p;
  for (int i = 0; i < _Dimension; ++i)
    p(i) = point[i];
  return addData(p);
}

/**
 * double addData(const _Scalar[] point)
 *
 * Receives a _Scalar array to be used as a data point. If the input array 
 * does not have a length _Dimension, memory will be grabbed that does not
 * belong to the array, which will lead to undefined behavior. 
 * @param point The _Scalar array that contains the data point to add.
 * @return The fraction of the stored data matrix that is used.
 */
template <typename _Scalar, int _Dimension>
double CovarianceTracker<_Scalar, _Dimension>::addData(const _Scalar point[])
{
  Eigen::Matrix<_Scalar, _Dimension, 1> p;
  for (int i = 0; i < _Dimension; ++i)
    p(i) = point[i];
  return addData(p);
}


#endif // COVARIANCETRACKER_CPP
