/*
 * IResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IRESAMPLER_H_
#define SRC_IRESAMPLER_H_

#include "Filter.h"

namespace ohmPf
{

  class IResampler
  {
  public:
    IResampler(){};
    virtual ~IResampler(){};
    virtual void resample(Filter* filter) = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_IRESAMPLER_H_ */
