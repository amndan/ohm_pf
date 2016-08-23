/*
 * LowVarianceResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LVRESAMPLER_H_
#define INCLUDE_LVRESAMPLER_H_

#include "IResampler.h"
#include "Filter.h"
#include "assert.h"
#include "IOCSClient.h"

namespace ohmPf
{

/**
 * @brief Implementation of a low variance resampler algorithm.
 */
class LVResampler : public IResampler
{
public:
  /**
   * @brief Constructor
   */
  LVResampler();

  /**
   * @brief Deconstructor (empty)
   */
  virtual ~LVResampler(){};

  /**
   * @brief Resampling function.
   * @param filter Filter instance --> Holds the particles to be resampled
   */
  void resample(Filter* filter);

  /**
   * @brief OCS flag management.
   * @todo Do we use OCS flag here?
   */
  void setOCSFlagTrue();

private:

  bool _OCSFlag;
};

} /* namespace ohmPf */

#endif /* INCLUDE_LVRESAMPLER_H_ */
