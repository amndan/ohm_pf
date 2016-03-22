/*
 * LowVarianceResampler.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "../include/LVResampler.h"

namespace ohmPf
{

  LVResampler::LVResampler()
  {
    // TODO Auto-generated constructor stub

  }

  LVResampler::~LVResampler()
  {
    // TODO Auto-generated destructor stub
  }

  void LVResampler::resample(Filter* filter)
  {

    SampleSet* set = filter->getSampleSet();
    std::vector<Sample_t>* samples = set->getSamples();
    unsigned int cnt = set->getCountSamples();

    // todo: use a more intelligent way to do this
    if(!set->isNormalized())
      set->normalize();

    std::vector<double> weightsCumsum;

    weightsCumsum.reserve(cnt);

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      weightsCumsum.push_back(it->weight);
    }

    //std::cout << "stabw:" << getStabw(weightsCumsum) << std::endl;

    std::partial_sum(weightsCumsum.begin(), weightsCumsum.end(), weightsCumsum.begin());

    std::vector<Sample_t> newSamples;
    newSamples.reserve(cnt);

    double rand;
    //low variance resampling
    unsigned int lowVarDist = cnt / 3;  //TODO: magic numbers; add launchfile parameter!
    unsigned int count = std::floor(cnt / lowVarDist);

    assert(count > 0);

    for(unsigned int i = 0; i < cnt;)
    {
      rand = drand48();  // rand [0;1] --> probs are normalized
      for(int j = 0; j < cnt; j++)
      {
        if(rand < weightsCumsum[j])  // search for matching prob
        {
          for(unsigned int k = 0; k < count && i < cnt; k++, i++)
          {
            unsigned int index = (j + k * lowVarDist) % cnt;  // prevent overflow
            newSamples.push_back(samples->at(index));
            addGaussianRandomness(newSamples[i]);  //TODO: variance of gaussian randomness as launchfileparam
            newSamples[i].weight = 1.0;
          }
          break;
        }
      }
    }

    assert(cnt == newSamples.size());
    set->setSamples(newSamples);
    set->normalize();
  }

} /* namespace ohmPf */

