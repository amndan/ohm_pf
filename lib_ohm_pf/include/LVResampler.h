/*
 * LowVarianceResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LVRESAMPLER_H_
#define INCLUDE_LVRESAMPLER_H_

#include "Filter.h"
#include "FilterUpdaterTimed.h"
#include "assert.h"
#include "UtilitiesOhmPf.h"
#include "ros/time.h"
#include <cmath>
#include "OCSClient.h"

namespace ohmPf
{

/**
 * @brief Implementation of a low variance resampling algorithm.
 */
class LVResampler : public FilterUpdaterTimed , public OCSClient
{
public:
  /**
   * @brief Constructor
   * @param addNoiseSigmaTrans Additional translational noise to increase filters variance at resampling step.
   * @param addNoiseSigmaRot Additional rotational noise to increase filters variance at resampling step.
   * @param lowVarianceFactor Each randomly chosen sample generates lowVarianceFactor new samples.
   */
  LVResampler(double addNoiseSigmaTrans, double addNoiseSigmaRot, unsigned int lowVarianceFactor, Filter* filter);

  /**
   * @brief Deconstructor (empty)
   */
  virtual ~LVResampler(){};



private:

  /**
   * @brief Resampling function.
   * @param filter Filter instance --> Holds the particles to be resampled
   * @todo normalizing shouold be done before or after accessing samples.
   * Each function must normalize with that rule!
   */
  void update();

  double _addNoiseSigmaRot;
  double _addNoiseSigmaTrans;
  unsigned int _lowVarianceFactor;
};

} /* namespace ohmPf */

#endif /* INCLUDE_LVRESAMPLER_H_ */
