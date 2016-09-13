/*
 * Filter.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "Filter.h"

namespace ohmPf
{

Filter::Filter(FilterParams_t params) :
    _sampleSet(params.samplesMax)
{
  _params = params;
  _filterState.probPose = 0.0;
}

unsigned int Filter::getSamplesMax()
{
  return _params.samplesMax;
}

unsigned int Filter::getSamplesMin()
{
  return _params.samplesMin;
}

std::vector<Sample_t>* Filter::getSamples()
{
  return _sampleSet.getSamples();
}

SampleSet* Filter::getSampleSet()
{
  return &_sampleSet;
}

void Filter::setSamples(std::vector<Sample_t> samples)
{
  _sampleSet.setSamples(samples);
}

FilterState_t* Filter::getFilterState()
{
  return &_filterState;
}

FilterParams_t Filter::getParams()
{
  return _params;
}

} /* namespace ohmPf */
