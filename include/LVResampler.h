/*
 * DefaultResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LVRESAMPLER_H_
#define INCLUDE_LVRESAMPLER_H_

#include "IResampler.h"
#include "Filter.h"
#include "assert.h"

namespace ohmPf
{

  class LVResampler : public IResampler
  {
  public:
    LVResampler();
    virtual ~LVResampler();
    void resample(Filter* filter);
  };

} /* namespace ohmPf */

#endif /* INCLUDE_LVRESAMPLER_H_ */
