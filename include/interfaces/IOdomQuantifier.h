/*
 * IOdomQuantifier.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_IODOMQUANTIFIER_H_
#define SRC_IODOMQUANTIFIER_H_

#include "IOdomMeasurement.h"
#include "Filter.h"

namespace ohmPf
{

  class IOdomQuantifier
  {
  public:
    IOdomQuantifier(){};
    virtual ~IOdomQuantifier(){};
    virtual void calculate(Filter& filter, IOdomMeasurement& measurement) = 0;

  };

} /* namespace ohmPf */

#endif /* SRC_IODOMQUANTIFIER_H_ */
