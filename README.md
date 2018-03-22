# covariance-tracker
Computes the covariance matrix in a window of incoming values. 

Inspiration:
http://rebcabin.github.io/blog/2013/01/22/covariance-matrices/

Methods defined:

### `CovarianceTracker<typename _Scalar, int _Dimension>(int len = 100)`
Default constructor. The covariance values are set to 0. Data length defaults to 100. <br />
`_Scalar` -- the datatype you will use to input your values, e.g. `double` or `float`. <br />
`_Dimension` -- an int equal to the number of variables in this tracker. E.g. for storing 
the x, y, and z values obtained from a 3-axis accelerometer, use `_Dimension = 3`. 

### `double addData(Eigen::Matrix<_Scalar, _Dimension, 1> point)`
Adds the specified data point to this tracker. Example:
<pre>
CovarianceTracker&lt;float, 3&gt; covtrack(100);
Eigen::Matrix&lt;float, 1, 3&gt; datapoint;  // datapoint is a 3-vector
datapoint &lt;&lt; 1.0, 2.0, 3.0;  // datapoint is now &lt;1.0, 2.0, 3.0&gt;
covtrack.addData(datapoint);
</pre>

### `double addData(const std::vector<_Scalar> point)`
Adds the specified data point to this tracker. Asserts the size of point
is equal to `_Dimension`! So only pass the correct number of arguments or
you will get a runtime error.


### `double addData(const _Scalar[] point)`
Receives a `_Scalar` array to be used as a data point. If the input array 
does not have a length `_Dimension`, memory will be grabbed that does not
belong to the array, which will lead to undefined behavior. 
Returns the fraction of the stored data matrix that is used.


### `Eigen::Matrix<double, _Dimension, 1> getMean(void)`
Returns the mean vector of the values stored in this covariance tracker.


### `Eigen::Matrix<double, _Dimension, _Dimension> getCovariance(void)`
Calculate and return the covariance matrix. If no data or one datum has been inserted 
into this tracker, returns a `_Dimension` x `_Dimension` matrix of zeros. 


### `int getDataLength(void)`
Returns the number of data that can be stored in this tracker. Unfortunately, there is
no way to change this value, so if you want to change it, you'll just have to initialize
a new `CovarianceTracker` object. 


### `int getDimension(void)`
Returns `_Dimension`.


### `double getFractionUsed(void)`
Returns the fraction of the data matrix that is used. (&gt;= 0 and &lt;= 1)
