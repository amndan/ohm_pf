/*
 * DefaultResampler.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "../include/STDResampler.h"

namespace ohmPf
{

  STDResampler::STDResampler()
  {
    // TODO Auto-generated constructor stub

  }

  STDResampler::~STDResampler()
  {
    // TODO Auto-generated destructor stub
  }

  void STDResampler::resample(Filter* filter)
  {
    SampleSet* set = filter->getSampleSet();
    std::vector<Sample_t>* samples = set->getSamples();
    unsigned int countSamples = set->getCountSamples();

    // todo: use a more intelligent way to do this
    if(!set->isNormalized()) set->normalize();

    std::vector<double> weightsCumsum;

    weightsCumsum.reserve(countSamples);

    for(int i = 0; i < samples->size(); i++)
    {
      weightsCumsum.push_back(samples->at(i).weight);
    }

    //std::cout << "stabw:" << getStabw(weightsCumsum) << std::endl;

    std::partial_sum(weightsCumsum.begin(), weightsCumsum.end(), weightsCumsum.begin());

    std::vector<Sample_t> newSamples;
    newSamples.reserve(countSamples);

    double rand;

    for(unsigned int i = 0; i < countSamples; i++)
    {
      rand = drand48();
      for(int j = 0; j < countSamples; j++)
      {
        if(rand < weightsCumsum[j])
        {
          newSamples.push_back(samples->at(j));
          addGaussianRandomness(newSamples[i]);
          newSamples[i].weight = 1.0;
          break;
        }
      }
    }

    *samples = newSamples;
    set->normalize();


  }

} /* namespace ohmPf */

