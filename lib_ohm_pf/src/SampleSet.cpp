/*
 * SampleSet.cpp
 *
 *  Created on: Jan 5, 2016
 *      Author: amndan
 */

#include "SampleSet.h"

namespace ohmPf
{

  SampleSet::SampleSet(unsigned int numSamples)
  {
    assert(numSamples > 2);

    Sample_t sample;
    sample.pose(0) = 0;
    sample.pose(1) = 0;
    sample.pose(2) = 0;
    sample.weight = 1;

    _samples.clear();

    for(unsigned int i = 0; i < numSamples; i++)
    {
      _samples.push_back(sample);
    }

    _countSamples = (int) numSamples;
    _normalized = false;

    // initialize seed
    srand48(std::time(NULL));
  }

  SampleSet::~SampleSet()
  {
    // TODO Auto-generated destructor stub
  }

  std::vector<Sample_t>* SampleSet::getSamples()
//TODO: implement a getSamplesConst function withot setting norm to false
  {
    _normalized = false;
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

  void SampleSet::boostWeights()
  {
    for(unsigned int i = 0; i < _samples.size(); i++)
    {
      _samples[i].weight = std::pow(_samples[i].weight, 1.0 / 3.0);
      // TODO: magic numbers; 3rd sqrt leads to slower convergence
      // should be done in resampling step, not at normalization!
      // the more often it is done the slower the convergence
    }

    _normalized = false;
  }

  void SampleSet::normalize()
  {
    double sumWeight = 0.0;
    double countInv = 1.0 / _samples.size();

    for(unsigned int i = 0; i < _samples.size(); i++)
    {
      sumWeight += _samples[i].weight;
    }

	if (sumWeight != 0) // e.g. if map update while sample cloud is outside the map
	{
		for (unsigned int i = 0; i < _samples.size(); i++) {
			_samples[i].weight /= sumWeight;
		}
	}
	else
	{
		for (unsigned int i = 0; i < _samples.size(); i++) {
			_samples[i].weight = countInv;
		}
	}

    _normalized = true;
  }

  bool SampleSet::isNormalized()
  {
    return _normalized;
  }

} /* namespace ohmPf */
