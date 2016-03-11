/*
 * SampleSet.cpp
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#include "SampleSet.h"

namespace ohmPf
{

SampleSet::SampleSet(std::vector<Sample_t> samples)
{
  _samples = samples;
  _countSamples = samples.size();
  _normalized = false;
  srand48(std::time(NULL));
}

SampleSet::~SampleSet()
{
  // TODO Auto-generated destructor stub
}

std::vector<Sample_t>* SampleSet::getSamples()
{
  _normalized = false; //todo: bad implementation here
  return &_samples;
}

void SampleSet::setSamples(std::vector<Sample_t> samples)
{
  if(samples.size() > 1)
  {
    _samples = samples;
    _countSamples = _samples.size();
    _normalized = false;
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "minimum sample count is 2!" << std::endl;
    exit(EXIT_FAILURE);
  }
}

int SampleSet::getCountSamples() const
{
  return _countSamples;
}

} /* namespace ohmPf */
