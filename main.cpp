// Main method?

#include <iostream>
#include <cstdio>
//#include <>
#include "covariance-tracker.h"

int main()
{
  CovarianceTracker<double, 3> cov;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      cov.covariance(i,j) = i*j;
      printf("%.2f ", cov.covariance(i,j));
    }
    std::cout << std::endl;
  }
  return 0;
}