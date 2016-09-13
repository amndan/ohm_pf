/*
 * IResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IRESAMPLER_H_
#define SRC_IRESAMPLER_H_

#include "Filter.h"
#include "OCSClient.h"

namespace ohmPf
{

/**
 * @brief Interface class for a resampling algorithm.
 * Inherits from IOCSClient to provide OCS functionality.
 */
class IResampler : public OCSClient , public FilterUpdater
{
public:

  /**
   * @brief interface constructor (empty).
   */
  IResampler(){};

  /**
   *  @brief interface destructor (empty).
   */
  virtual ~IResampler(){};

  /**
   * @brief Abstract resampling function.
   * Resamples the particlecloud of the given filter instance.
   * @param filter pointer to a filter instance
   */
  virtual void update() = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IRESAMPLER_H_ */
