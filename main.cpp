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
  CovarianceTracker<double, 3> covTrack(15);
  double used;
  for(double i = 0.0; i < 3.0*15.0; i+=3.0) {
    used = 100.0*covTrack.addData(i,i+1.0,i+2.0);
    printf("Inserting (%.2f, %.2f, %.2f)...\n", i, i+1.0, i+2.0);
    std::cout << "Used: " << used << "%" << std::endl;

    auto cov = covTrack.getCovariance();
    std::cout << cov << std::endl;
  }

  std::cout << "Mean: " << std::endl << covTrack.getMean() << std::endl;

  /*auto cov = covTrack.getCovariance();
  
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      printf("%.2f ", cov(i,j));
    }
    std::cout << std::endl;
  }*/
  return 0;
}