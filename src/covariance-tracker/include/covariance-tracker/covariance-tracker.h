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
 * @version 0.1?
 * @brief Calculate the covariance of a set of X-dimensional values.
 */

#ifndef COVARIANCETRACKER_H
#define COVARIANCETRACKER_H

#if __cplusplus <= 199711L
  #error This library needs at least C++11! Compile with -std=c++11 or gnu++11.
#endif

#include <Eigen/Dense>
#include <vector>


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
  //double addData(_Scalar a1, ...);
  
  /**
   * double addData(const std::vector<_Scalar> point)
   *
   * Adds the specified data point to this tracker. Asserts the size of point
   * is equal to _Dimension! So only pass the correct number of arguments or
   * you will get a runtime error. 
   * @param point The std::vector<_Scalar> containing the data
   * @return The fraction of the stored data matrix that is used.
   */
  double addData(const std::vector<_Scalar> &point);

  /**
   * double addData(const _Scalar[] point)
   *
   * Receives a _Scalar array to be used as a data point. If the input array 
   * does not have a length _Dimension, memory will be grabbed that does not
   * belong to the array, which will lead to undefined behavior. 
   * @param point The _Scalar array that contains the data point to add.
   * @return The fraction of the stored data matrix that is used.
   */
  double addData(const _Scalar point[]);

  /**
   * int getDataLength(void)
   *
   * @return The number of data that can be stored in this tracker.
   */
  int getDataLength(void) const
  {
    return data_length_;
  }

  /**
   * Eigen::Matrix<>& getCovariance(void)
   * 
   * Calculate the covariance matrix. If no data has been inserted into this tracker,
   * returns a _Dimension x _Dimension matrix of zeros. 
   * @return The current calculated covariance matrix. 
   */
  Eigen::Matrix<double, _Dimension, _Dimension> getCovariance(void);

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
  Eigen::Matrix<double, _Dimension, 1> getMean(void);

  /* 
   * double getFractionUsed(void)
   *
   * @return The fraction of the data matrix that is used. (>= 0 and <= 1)
   */
  double getFractionUsed(void) const
  {
    return (static_cast<double>(num_used_data_) 
      / static_cast<double>(data_length_));
  }

private:
  bool update_mean_, update_cov_, update_residuals_;
  int newest_data_;  // The pointer to the newest value inserted.
  int num_used_data_;  // The number of data used.
  Eigen::Matrix<double, _Dimension, 1> mean_;
  const int data_length_;
  Eigen::Matrix<double, Eigen::Dynamic, _Dimension> data_double_;
  Eigen::Matrix<double, Eigen::Dynamic, _Dimension> residuals_;
  Eigen::Matrix<double, _Dimension, _Dimension> covariance_;
  // The column matrices for each dimension; i.e. columns_[0] contains the matrix
  //  with all row[0] values equal to 1 and all other values equal to 0.
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, _Dimension> > columns_;

  /**
   * Eigen::Matrix<double, Eigen::Dynamic, _Dimension> getResiduals(void)
   *
   * Updates residuals_, so returns nothing. 
   */
  void calculateResiduals(void);
};

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
// #error This library needs at least C++11! Compile with -std=c++11 or gnu++11.
//#endif


/**
 * Constructor. The covariance values are set to 0. Data length defaults to 100.
 */
template <typename _Scalar, int _Dimension>
CovarianceTracker<_Scalar, _Dimension>::CovarianceTracker(int len)
  : update_mean_(false),
    update_cov_(false),
    update_residuals_(false),
    newest_data_(-1),
    num_used_data_(0),
    mean_(),
    data_length_(len),
    data_double_(len, _Dimension),
    residuals_(len, _Dimension),
    covariance_(Eigen::Matrix<double, _Dimension, _Dimension>::Zero()),
    columns_(_Dimension, 
      Eigen::Matrix<double, Eigen::Dynamic, _Dimension>::Zero(len, _Dimension))
{
  // Set the columns_ vector matrices to 1-columns (see header description)
  for (int i = 0; i < _Dimension; ++i)
  {
    for (int j = 0; j < len; ++j)
    {
      columns_[i](j, i) = 1.0;
    }
  }
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
  ++newest_data_;
  newest_data_ %= data_length_;

  // keep increasing num_used_data_ unless we have reached maximum
  if (num_used_data_ < data_length_)
    ++num_used_data_;

  // insert new data into the matrix
  for (int i = 0; i < _Dimension; ++i)
    data_double_(newest_data_, i) = static_cast<double>(point(i));

  // For debugging.
  //std::cout << data_ << std::endl;
  //std::cout << residuals_ << std::endl;
  
  // alert return functions that the data has changed
  update_mean_ = true;
  update_cov_ = true;
  update_residuals_ = true;

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
::addData(const std::vector<_Scalar> &point)
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

/**
 * Eigen::Matrix<>& getCovariance(void)
 * 
 * Calculate the covariance matrix. If no data or one datum has been inserted 
 * into this tracker, returns a _Dimension x _Dimension matrix of zeros. 
 * @return The current calculated covariance matrix. 
 */
template <typename _Scalar, int _Dimension>
Eigen::Matrix<double, _Dimension, _Dimension> 
CovarianceTracker<_Scalar, _Dimension>::getCovariance(void)
{
  if (update_cov_ && num_used_data_ > 1) {
    // update fields, if we need to
    getMean();
    calculateResiduals();
    // calculate new covariance matrix
    update_cov_ = false;
    return covariance_ = 
                (residuals_.block(0, 0, num_used_data_, _Dimension).transpose()
                * residuals_.block(0, 0, num_used_data_, _Dimension)).eval()
                / (static_cast<double>(num_used_data_) - 1.0);
  } else {
    return covariance_;
  }
}

/**
 * Eigen::Matrix<double, _Dimension, 1> getMean(void)
 *
 * @return The mean vector of the values stored in this covariance tracker.
 */
template <typename _Scalar, int _Dimension>
Eigen::Matrix<double, _Dimension, 1>
CovarianceTracker<_Scalar, _Dimension>::getMean(void)
{
  if (update_mean_) {
    for (int i = 0; i < _Dimension; ++i) {
      mean_(i) = data_double_.block(0, 0, num_used_data_, _Dimension)
                 .col(i)
                 .sum()
                 / static_cast<double>(num_used_data_);
    }
    update_mean_ = false;

    // Debugging
    //std::cout << mean_ << std::endl;
    
    return mean_;
  } else {
    return mean_;
  }
}


template <typename _Scalar, int _Dimension>
void CovarianceTracker<_Scalar, _Dimension>::calculateResiduals(void)
{
  if (update_residuals_) {
    // create tmp_means, a matrix with the mean of column x filling the x column
    Eigen::Matrix<double, Eigen::Dynamic, _Dimension> 
    tmp_means(data_length_, _Dimension);
    tmp_means = 
      Eigen::Matrix<double, Eigen::Dynamic, _Dimension>
      ::Zero(data_length_, _Dimension);
    for (int i = 0; i < _Dimension; ++i) {
        tmp_means += (columns_[i] * mean_(i));
    }

    residuals_ = data_double_ - tmp_means;
    update_residuals_ = false;

    //std::cout << residuals_ << std::endl;
  }
}

#endif // COVARIANCETRACKER_CPP

#endif //COVARIANCETRACKER_H
