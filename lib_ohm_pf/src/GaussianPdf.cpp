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

Sample_t GaussianPdf::getRandomSample(Eigen::Vector3d mu, double sigmaT, double sigmaPhi)
{
  Sample_t sample;

  sample.pose(0) = getRandomValue(mu(0), sigmaT);
  sample.pose(1) = getRandomValue(mu(1), sigmaT);
  sample.pose(2) = getRandomValue(mu(2), sigmaPhi);
  sample.weight = 1.0;

  return sample;
}

double GaussianPdf::getProbability(double mu, double sigma, double x)
{
  if (sigma <= 0)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> sigma <= 0; exit!";
    exit(EXIT_FAILURE);
  }

  //todo: precalculate density with normalized gaussian
  return (1) / (sqrt(2 * M_PI * pow(sigma, 2))) * pow(M_E, ((-0.5 * pow((x - mu), 2)) / (pow(sigma, 2))));
}

double GaussianPdf::getRandomValue(double mu, double sigma)
{
  if (sigma < 0)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> sigma < 0; sigma = " << sigma << "; exit!";
    exit(EXIT_FAILURE);
  }

  if (sigma == 0)
  {
    return mu;
  }

  if (sigma > 0)
  {
    if (!_initialized)
      initializeSeed();

    double tmp = 0.0;

    // calculating a sample with a for loop --> see book probablistic robotics by S. Thrun et al.
    for (unsigned int i = 0; i < 12; i++)
    {
      tmp += drand48() * 2.0 * sigma - sigma;
    }
    return 0.5 * tmp + mu;
  }

  std::cout << __PRETTY_FUNCTION__ << "--> sigma != real; sigma = " << sigma << "; exit!";
  exit(EXIT_FAILURE);
}

void GaussianPdf::initializeSeed()
{
  srand48(std::time(NULL)); // initialize random number generator
  _initialized = true; // just do that one times
}

} /* namespace ohmPf */
