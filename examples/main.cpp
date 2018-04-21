/** 
 * Main method?
 * @author Joshua Petrin
 */

#include <iostream>
#include <cstdio>
//#include <>
#include "covariance-tracker.h"

typedef float ttype;

int main()
{
  CovarianceTracker<ttype, 3> covTrack(15);
  double used;
  for(ttype i = ttype(0.0); i < 3.0*45.0; i+=ttype(3.0)) {
    used = 100.0*covTrack.addData(
      std::vector<ttype> {i, ttype(log(i+1.0)), i+ttype(2.0)});
    //printf("Inserted {%.2f, %.2f, %.2f}.\n", i, float(log(i+1.0f)), i+2.0f);
    std::cout << "Used: " << used << "%" << std::endl;

    Eigen::Matrix<double, 3, 3> cov = covTrack.getCovariance();
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