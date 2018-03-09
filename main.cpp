/** 
 * Main method?
 * @author Joshua Petrin
 */

#include <iostream>
#include <cstdio>
//#include <>
#include "covariance-tracker.h"

int main()
{
  CovarianceTracker<double, 3> covTrack;
  auto cov = covTrack.getCovariance();
  
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      printf("%.2f ", cov(i,j));
    }
    std::cout << std::endl;
  }
  return 0;
}