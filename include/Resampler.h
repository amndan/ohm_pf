/*
 * Resampler.h
 *
 * Created on: Tue Feb 23 13:04:30 CET 2016
 * Author: amndan
 *
 */

#ifndef RESAMPLER_H_
#define RESAMPLER_H_


#include "Sensor.h"
#include "Filter.h"

namespace ohmPf
{

  class Resampler : public Sensor
  {
    public:
      Resampler();
      void updateFilter(Filter& filter);
      void initFilter(Filter& filter);
      virtual ~Resampler(){};
  };

} //namespace OhmPf

#endif //RESAMPLER_H_
