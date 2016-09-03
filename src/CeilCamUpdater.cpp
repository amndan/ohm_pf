/*
 * CeilCam.cpp
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#include "../include/CeilCamUpdater.h"

namespace ohmPf
{

  CeilCamUpdater::CeilCamUpdater(Filter* filter, ICeilCamMeasurement* measurement, MapUpdater* updateFilterMap) :
          FilterUpdater(filter)
  {
    _measurement = measurement;
    _updateFilterMap = updateFilterMap;
  }

  void CeilCamUpdater::injectSamples()
  {
    std::vector<Sample_t>* samples = _filter->getSamples();

    double weightAvg = 0;

    for(int i = 0; i < samples->size(); i++)
    {
      weightAvg += samples->at(i).weight;
    }
    weightAvg /= samples->size();

    double maxWeight = weightAvg / 3.0;
    int countNewSamples = 0;

    for(std::vector<Sample_t>::iterator iter = samples->begin(); iter < samples->end(); iter++)
    {
      if(iter->weight < maxWeight)
      {
        samples->erase(iter);
        countNewSamples++;
      }
    }

//    for(int i = 0; i < samples->size(); i++)
//    {
//      if(samples->at(i).weight < maxWeight)
//      {
//        samples->erase(samples->begin() + i);
//        countNewSamples++;
//      }
//    }

    while(countNewSamples > 0)
    {
      for(int i = 0; i < _measurement->getPoses().size(); i++)
      {
        Sample_t newSample;
        newSample.weight = weightAvg;
        newSample.pose = _measurement->getPoses().at(i);
        addGaussianRandomness(newSample, 0.5, 10 / 180 * M_PI);
        samples->push_back(newSample);
        countNewSamples--;
        if(countNewSamples <= 0) break;
      }
    }
  }

  void CeilCamUpdater::update()
  {
    // todo: better implementation
    std::vector<Sample_t>* samples = _filter->getSamples();
    std::vector<double> probs(samples->size(), 0.0);

    std::vector<Eigen::Vector3d> poses = _measurement->getPoses();

    for(int i = 0; i < poses.size(); i++)
    {
      for(int k = 0; k < samples->size(); k++)
      {
        probs[k] = std::max(probs[k], getProbabilityFrom2Poses(poses[i], samples->at(k).pose, 2.0));  // todo: magic number
      }
    }


    //    // at first find maximum weight eg minimum distance for each sample
    //    for(std::vector<Eigen::Vector3d>::iterator it = _measurement->getPoses().begin(); it != _measurement->getPoses().end(); ++it)
    //    {
    //      for(std::vector<Sample_t>::iterator it2 = samples->begin(); it2 != samples->end(); ++it2)
    //      {
    //        std::cout << "loop"  << std::endl;
    //
    //        probs[n] = std::max(probs[n], getProbabilityFrom2Poses(*it, it2->pose, 2.0)); // todo: magic number
    //      }
    //    }
    //

    // then multiply weights with old weights
    for(int k = 0; k < samples->size(); k++)
    {
      samples->at(k).weight *= probs.at(k);
    }

    //    // then multiply weights with old weights
    //    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    //    {
    //      int n = std::distance(samples->begin(), it);
    //      it->weight = it->weight * probs[n];
    //    }
    //
    //
    //    _filter->getSampleSet()->normalize();
    //    //_updateFilterMap->update();

    injectSamples();

    _filter->getSampleSet()->normalize();
    _updateFilterMap->update();
    _filter->getSampleSet()->normalize();

    //    //filter.getSampleSet()->resample();

  }
} /* namespace ohmPf */
