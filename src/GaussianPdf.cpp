/*
 * GaussianPdf.cpp
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#include "GaussianPdf.h"

namespace ohmPf
{
  // initialize static member variable
  bool GaussianPdf::_initialized = false;

  GaussianPdf::GaussianPdf()
  {

  }

  GaussianPdf::~GaussianPdf()
  {
    // TODO Auto-generated destructor stub
  }

  Sample_t GaussianPdf::getRandomSample(Sample_t mu, double sigmaT, double sigmaPhi)
  {
    Sample_t sample;

    sample.pose(0) = getRandomValue((double)mu.pose(0), sigmaT);
    sample.pose(1) = getRandomValue((double)mu.pose(1), sigmaT);
    sample.pose(2) = getRandomValue((double)mu.pose(2), sigmaPhi);

    return sample;
  }

  double GaussianPdf::getRandomValue(double mu, double sigma)
  {
    assert(sigma > 0);

    if(!_initialized)
      initializeSeed();

    double tmp;

    for(unsigned int i = 0; i < 12; i++)
    {
      tmp += drand48() * 2.0 * sigma - sigma;
    }

    return 0.5 * tmp + mu;
  }

  void GaussianPdf::initializeSeed()
  {
    srand48(std::time(NULL));
    _initialized = true;
  }

} /* namespace ohmPf */
