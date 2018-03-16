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
  CovarianceTracker<float, 3> covTrack(15);
  double used;
  for(float i = 0.0; i < 3.0f*45.0f; i+=3.0f) {
    used = 100.0*covTrack.addData(std::vector<float> {i, float(log(i+1.0f)), i+2.0f});
    printf("Inserted {%.2f, %.2f, %.2f}.\n", i, log(i+1.0f), i+2.0f);
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