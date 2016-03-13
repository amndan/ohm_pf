/*
 * CeilCam.cpp
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#include "../include/CeilCamUpdater.h"

namespace ohmPf
{

  CeilCamUpdater::CeilCamUpdater(Filter* filter, ICeilCamMeasurement* measurement, IUpdateFilterMap* updateFilterMap) :
      FilterUpdater(filter)
  {
    _measurement = measurement;
    _updateFilterMap = updateFilterMap;
  }

  CeilCamUpdater::~CeilCamUpdater()
  {
    // TODO Auto-generated destructor stub
  }

  void CeilCamUpdater::update()
  {
    // todo: better implementation
    std::vector<Sample_t>* samples = _filter->getSamples();
    std::vector<double> probs(samples->size(), 0.0);

    // at first find maximum weight eg minimum distance for each sample
    for(std::vector<Eigen::Vector3d>::iterator it = _measurement->getPoses().begin(); it != _measurement->getPoses().end(); ++it)
    {
      for(std::vector<Sample_t>::iterator it2 = samples->begin(); it2 != samples->end(); ++it2)
      {
        int n = std::distance(samples->begin(), it2);
        probs[n] = std::max(probs[n], getProbabilityFrom2Poses(*it, it2->pose, 2.0)); // todo: magic number
      }
    }

    // then multiply weights with old weights
    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      int n = std::distance(samples->begin(), it);
      it->weight = it->weight * probs[n];
    }

    _updateFilterMap->update();
    //filter.getSampleSet()->normalize();
    //filter.getSampleSet()->resample();
  }
} /* namespace ohmPf */
