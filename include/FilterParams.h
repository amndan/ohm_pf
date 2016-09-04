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

/**
 * @brief structure for holding filter parameters.
 */
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
  double minValidScanRaysFactor;
  double resamplerAdditionalTranslationalNoise;
  double resamplerAdditionalRotationalNoise;
  unsigned int resamplerLowVarianceFactor;
  double OCSRotToTransFactor;
  unsigned int maxDistanceProbMap;
} FilterParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_FilterParams_H_ */
