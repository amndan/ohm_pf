/*
 * Resampler.cpp
 *
 * Created on: Tue Feb 23 13:04:30 CET 2016
 * Author: amndan
 *
 */

#include "Resampler.h"

namespace ohmPf
{
  Resampler::Resampler()
  {
  
  }

  void Resampler::updateFilter(Filter& filter)
  {
    filter.getSampleSet()->normalize();
    filter.getSampleSet()->resample();
  }

  void Resampler::initFilter(Filter& filter)
  {
    std::cout << __PRETTY_FUNCTION__ << "Resampler is not able to init Filter --> skipping" << std::endl;
  }

}
