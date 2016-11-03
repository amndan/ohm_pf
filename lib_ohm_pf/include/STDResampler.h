/*
 * DefaultResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_STDRESAMPLER_H_
#define INCLUDE_STDRESAMPLER_H_

#include "Filter.h"
#include "assert.h"
#include "FilterUpdaterTimed.h"
#include "UtilitiesOhmPf.h"
#include <cmath>
#include "OCSClient.h"

namespace ohmPf
{

/**
 * @brief An implementation of a standard random resampling algorithm.
 */
class STDResampler : public FilterUpdaterTimed , public OCSClient
{
public:
  /**
   * @brief Constructor
   * @param addNoiseSigmaTrans Additional translational noise to increase filters variance at resampling step.
   * @param addNoiseSigmaRot Additional rotational noise to increase filters variance at resampling step.
   */
  STDResampler(double addNoiseSigmaTrans, double addNoiseSigmaRot, Filter* filter, std::string idString);

  /**
   * @brief Deconstructor (empty)
   */
  virtual ~STDResampler(){};

private:
  /**
   * @brief Resampling function.
   * @param filter Filter instance --> Holds the particles to be resampled
   */
  void update();

  double _addNoiseSigmaRot;
  double _addNoiseSigmaTrans;
  double _minStabwToResample;
  bool _useAdaptiveMean;
};

} /* namespace ohmPf */

#endif /* INCLUDE_STDRESAMPLER_H_ */
