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
    _OCSFlag = false;
  }

  void LVResampler::resample(Filter* filter)
  {
    if(_OCSFlag == true)
    {
      SampleSet* set = filter->getSampleSet();
      std::vector<Sample_t>* samples = set->getSamples();
      unsigned int cnt = set->getCountSamples();

      set->normalize();

      std::vector<double> weightsCumsum;
      std::vector<Sample_t> newSamples;

      weightsCumsum.reserve(cnt);
      newSamples.reserve(cnt);

      for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
      {
        weightsCumsum.push_back(it->weight);
      }

      //std::cout << "stabw:" << getStabw(weightsCumsum) << std::endl;

      // create a cumulated sum vector
      std::partial_sum(weightsCumsum.begin(), weightsCumsum.end(), weightsCumsum.begin());

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
              newSamples[i].weight = 1.0 / cnt; // normalizing is included here
            }
            break;
          }
        }
      }

      assert(cnt == newSamples.size());

      set->setSamples(newSamples);

      _OCSFlag = false;
    }
  }

  void LVResampler::setOCSFlagTrue()
  {
    _OCSFlag = true;
  }

} /* namespace ohmPf */

