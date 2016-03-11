/*
 * Filter.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "Filter.h"

namespace ohmPf
{

  Filter::Filter(std::vector<Sample_t> samples, unsigned int samplesMin, unsigned int samplesMax) :
      _sampleSet(samples)
  {
    assert(samplesMin > 1);
    assert(samplesMin <= samplesMax);

    _samplesMax = samplesMax;
    _samplesMin = samplesMin;
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

  void Filter::setSamples(std::vector<Sample_t> samples)
  {
    _sampleSet.setSamples(samples);
  }

} /* namespace ohmPf */
