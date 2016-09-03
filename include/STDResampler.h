/*
 * DefaultResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_STDRESAMPLER_H_
#define INCLUDE_STDRESAMPLER_H_

#include "IResampler.h"
#include "IOCSClient.h"
#include "Filter.h"
#include "assert.h"
#include "UtilitiesOhmPf.h"
#include <cmath>

namespace ohmPf
{

/**
 * @brief An implementation of a standard random resampling algorithm.
 */
class STDResampler : public IResampler
{
public:
  /**
   * @brief Constructor
   * @param addNoiseSigmaTrans Additional translational noise to increase filters variance at resampling step.
   * @param addNoiseSigmaRot Additional rotational noise to increase filters variance at resampling step.
   */
  STDResampler(double addNoiseSigmaTrans, double addNoiseSigmaRot);

  /**
   * @brief Deconstructor (empty)
   */
  virtual ~STDResampler(){};

  /**
   * @brief Resampling function.
   * @param filter Filter instance --> Holds the particles to be resampled
   */
  void resample(Filter* filter);

  /**
   * @brief OCS flag management (see @ref OCS).
   */
  void setOCSFlagTrue();

private:
  bool _OCSFlag;
  double _addNoiseSigmaRot;
  double _addNoiseSigmaTrans;
};

} /* namespace ohmPf */

#endif /* INCLUDE_STDRESAMPLER_H_ */
