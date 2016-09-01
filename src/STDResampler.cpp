/*
 * DefaultResampler.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "../include/STDResampler.h"

#include <cstdlib>
#include <iterator>
#include <numeric>
#include <vector>

#include "../include/Sample.h"
#include "../include/SampleSet.h"
#include "../include/UtilitiesOhmPf.h"

namespace ohmPf
{

STDResampler::STDResampler(double addNoiseSigmaTrans, double addNoiseSigmaRot)
{
  _OCSFlag = false;
  _addNoiseSigmaRot = abs(addNoiseSigmaRot);
  _addNoiseSigmaTrans = abs(addNoiseSigmaTrans);
}

STDResampler::~STDResampler()
{
  // TODO Auto-generated destructor stub
}

void STDResampler::resample(Filter* filter)
{
  if(_OCSFlag == true)
  {
    SampleSet* set = filter->getSampleSet();
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

    //std::cout << "stabw:" << getStabw(weightsCumsum) << std::endl;

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
          addGaussianRandomness(newSamples[i], _addNoiseSigmaTrans, _addNoiseSigmaRot);
          newSamples[i].weight = 1.0 / countSamples;
          break;
        }
      }
    }

    assert(countSamples == newSamples.size());

    *samples = newSamples;
    _OCSFlag = false;
  }
}

void STDResampler::setOCSFlagTrue()
{
  _OCSFlag = true;
}

} /* namespace ohmPf */

