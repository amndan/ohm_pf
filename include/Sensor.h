/*
 * Sensor.h
 *
 *  Created on: 18.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_SENSOR_H_
#define INCLUDE_SENSOR_H_

//#include "Filter.h"

namespace ohmPf
{

  class Filter;

  class Sensor
  {
  public:
    Sensor(){};
    virtual void updateFilter(Filter& filter) = 0;
    virtual ~Sensor(){};
  };

} /* namespace ohmPf */

#endif /* INCLUDE_SENSOR_H_ */
