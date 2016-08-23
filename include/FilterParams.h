/*
 * FilterParams.h
 *
 *  Created on: Jan 8, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_FilterParams_H_
#define INCLUDE_FilterParams_H_

#include <string>

namespace ohmPf
{

typedef struct
{
  unsigned int samplesMin;
  unsigned int samplesMax;
  double resamplingIntervall;
  std::string resamplingMethod;
  double OCSThresholdLaser;
  double OCSThresholdOdom;
  double OCSThresholdResampler;
  unsigned int countLasers;
} FilterParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_FilterParams_H_ */
