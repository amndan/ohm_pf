/*
 * PoseUpdater.cpp
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#include "PoseUpdater.h"

namespace ohmPf
{

  PoseUpdater::PoseUpdater(Filter* filter, IPoseMeasurement* measurement, MapUpdater* updateFilterMap, std::string idString) :
          FilterUpdaterMeasurementOCS(measurement, filter, idString)
  {
    _poseMeasurement = measurement;
    _updateFilterMap = updateFilterMap;

    deactivateOCS();
  }

  void PoseUpdater::injectSamples()
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
      for(int i = 0; i < _poseMeasurement->getPoses().size(); i++)
      {
        Sample_t newSample;
        newSample.weight = weightAvg;
        newSample.pose = _poseMeasurement->getPoses().at(i);
        addGaussianRandomness(newSample, 0.5, 10 / 180 * M_PI);
        //addUniformRandomness(newSample, 3.0, 2 * M_PI);
        samples->push_back(newSample);
        countNewSamples--;
        if(countNewSamples <= 0) break;
      }
    }
  }

  void PoseUpdater::update()
  {
    injectSamples();

    // todo: better implementation
    std::vector<Sample_t>* samples = _filter->getSamples();
    std::vector<double> probs(samples->size(), 0.0);

    std::vector<Eigen::Vector3d> poses = _poseMeasurement->getPoses();

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
    //    //_updateFilterMap->tryToUpdate();



    _filter->getSampleSet()->normalize();
    _updateFilterMap->tryToUpdate();
    _filter->getSampleSet()->normalize();

    //    //filter.getSampleSet()->resample();

  }
} /* namespace ohmPf */
