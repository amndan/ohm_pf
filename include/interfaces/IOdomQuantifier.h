/*
 * IOdomQuantifier.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IODOMQUANTIFIER_H_
#define SRC_IODOMQUANTIFIER_H_

#include "IUpdateFilterMap.h"
#include "IOdomMeasurement.h"
#include "IFilter.h"

namespace ohmPf
{

  class IOdomQuantifier
  {
  public:
    IOdomQuantifier(){};
    virtual ~IOdomQuantifier(){};
    virtual void calculate(IFilter& filter, IOdomMeasurement& measurement, IUpdateFilterMap& updateFilterMap) = 0;

  };

} /* namespace ohmPf */

#endif /* SRC_IODOMQUANTIFIER_H_ */
