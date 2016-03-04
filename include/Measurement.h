/*
 * ISensor.h
 *
 *  Created on: 18.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_MEASUREMENT_H_
#define INCLUDE_MEASUREMENT_H_

//#include "Filter.h"

namespace ohmPf
{

  class Filter;

  class Measurement
  {
  public:
    Measurement(){};
    virtual void updateFilter(Filter& filter) = 0;
    virtual void initFilter(Filter& filter) = 0;
    virtual ~Measurement(){};
  };

} /* namespace ohmPf */

#endif /* INCLUDE_MEASUREMENT_H_ */
