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

void SampleSet::normalize()
{
  if (!_normalized)
  {
    double sumWeight = 0.0;
    for(unsigned int i = 0; i < _samples.size(); i++)
    {
      sumWeight += _samples[i].weight;
    }

    assert(sumWeight != 0); // todo: debug why sumweight can be 0

    for(unsigned int i = 0; i < _samples.size(); i++)
    {
      _samples[i].weight /= sumWeight;
    }

    _normalized = true;
  }
}

bool SampleSet::isNormalized() const
{
  return _normalized;
}

void SampleSet::resample()
{
  assert(_normalized);

  std::vector<double> weightsCumsum;
  int countSamples = _countSamples;

  weightsCumsum.reserve(countSamples);

  for(std::vector<Sample_t>::iterator it = _samples.begin(); it != _samples.end(); ++it)
  {
    weightsCumsum.push_back(it->weight);
  }

  std::partial_sum(weightsCumsum.begin(), weightsCumsum.end(), weightsCumsum.begin());

  std::vector<Sample_t> newSamples;
  newSamples.reserve(countSamples);

  double rand;

  for(unsigned int i = 0; i < countSamples; i++)
  {
    rand = drand48();
    for(int j = 0; j < _countSamples; j++)
    {
      if(rand < weightsCumsum[j])
      {
        newSamples.push_back(_samples[j]);
        break;
      }
    }
  }

  _samples = newSamples;
  _normalized = false;
  _countSamples = countSamples;

}

} /* namespace ohmPf */
