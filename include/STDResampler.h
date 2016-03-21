/*
 * DefaultResampler.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef INCLUDE_STDRESAMPLER_H_
#define INCLUDE_STDRESAMPLER_H_

#include "IResampler.h"
#include "Filter.h"
#include "assert.h"

namespace ohmPf
{

  class STDResampler : public IResampler
  {
  public:
    STDResampler();
    virtual ~STDResampler();
    void resample(Filter* filter);
  };

} /* namespace ohmPf */

#endif /* INCLUDE_STDRESAMPLER_H_ */
