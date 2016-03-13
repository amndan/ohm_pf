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
    _samplesMax = params.samplesMax;
    _samplesMin = params.samplesMin;
  }

  Filter::~Filter()
  {
    // TODO Auto-generated destructor stub
  }

  unsigned int Filter::getSamplesMax()
  {
    return _samplesMax;
  }

  unsigned int Filter::getSamplesMin()
  {
    return _samplesMin;
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

} /* namespace ohmPf */
