/*
 * DefaultResampler.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "STDResampler.h"

namespace ohmPf
{

STDResampler::STDResampler(double addNoiseSigmaTrans, double addNoiseSigmaRot, Filter* filter, std::string idString) :
    FilterUpdaterTimed(filter, ros::Duration(filter->getParams().resamplingIntervall), idString)
{
  _addNoiseSigmaRot = std::abs(addNoiseSigmaRot);
  _addNoiseSigmaTrans = std::abs(addNoiseSigmaTrans);
  _useAdaptiveMean = filter->getParams().useAdaptiveMean;
  _minStabwToResample = filter->getParams().minStabwToResample;
}


void STDResampler::update()
{

  if(this->getOCSFlag() == true)
  {
    SampleSet* set = _filter->getSampleSet();
    std::vector<Sample_t>* samples = set->getSamples();
    unsigned int countSamples = set->getCountSamples();

    assert(countSamples > 0);

    set->normalize();

    std::vector<double> weightsCumsum;
    std::vector<Sample_t> newSamples;

    weightsCumsum.reserve(countSamples);
    newSamples.reserve(countSamples);
    for(int i = 0; i < samples->size(); i++)
    {
      weightsCumsum.push_back(samples->at(i).weight);
    }

    _filter->getFilterState()->stabWeights = getStabw(weightsCumsum);
    _filter->getFilterState()->probPose = getQualityOfSamples(*(_filter->getSamples()));

    if(_filter->getFilterState()->stabWeights < _minStabwToResample)
    {
      return; // dont resample
    }

    //calculate additional randomness factor

    double addRand;
    if(_useAdaptiveMean)
    {
      addRand = 0.5 * (1.0 - _filter->getFilterState()->probPose) + 0.5 * _filter->getFilterState()->adaptiveMeanQuotient;

      if(addRand < 0 || addRand > 1)
      {
        std::cout << __PRETTY_FUNCTION__ << "--> warning addRandFactor is not [0;1] its: " << addRand << std::endl;
      }
    }
    else
    {
      addRand = 1.0;
    }

    std::partial_sum(weightsCumsum.begin(), weightsCumsum.end(), weightsCumsum.begin());

    double rand;

    for(unsigned int i = 0; i < countSamples; i++)
    {
      rand = drand48();
      for(int j = 0; j < countSamples; j++)
      {
        if(rand < weightsCumsum[j])
        {
          newSamples.push_back(samples->at(j));
          addGaussianRandomness(newSamples[i], _addNoiseSigmaTrans * addRand, _addNoiseSigmaRot * addRand);
          addGaussianRandomness(newSamples[i], _addNoiseSigmaTrans * addRand, _addNoiseSigmaRot * addRand);
          newSamples[i].weight = 1.0 / countSamples;
          break;
        }
      }
    }

    assert(countSamples == newSamples.size());

    *samples = newSamples;
  }
}

} /* namespace ohmPf */

