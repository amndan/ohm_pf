/*
 * CeilCam.cpp
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#include "CeilCam.h"

namespace ohmPf
{

  CeilCam::CeilCam()
  {
    // TODO Auto-generated constructor stub

  }

  void CeilCam::initFilter(Filter& filter)
  {
    std::cout << "not yet implemented" << std::endl;
    //todo:: imlement init filter here
  }

  CeilCam::~CeilCam()
  {
    // TODO Auto-generated destructor stub
  }

  void CeilCam::setMeasurement(std::vector<Eigen::Vector3d> measurement)
  {
    _measurement = measurement;
  }

  void CeilCam::updateFilter(Filter& filter)
  {
    // todo: better implementation
    std::vector<Sample_t>* samples = filter.getSampleSet()->getSamples();
    std::vector<double> probs(samples->size(), 0.0);

    for(std::vector<Eigen::Vector3d>::iterator it = _measurement.begin(); it != _measurement.end(); ++it)
    {
      for(std::vector<Sample_t>::iterator it2 = samples->begin(); it2 != samples->end(); ++it2)
      {
        int n = std::distance(samples->begin(), it2);
        probs[n] = std::max(probs[n], getProbabilityFrom2Poses(*it, it2->pose, 2.0)); // todo: magic number
      }
    }

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      int n = std::distance(samples->begin(), it);
      it->weight = it->weight * probs[n];
    }

    filter.updateWithSensor(MAP);
    //filter.getSampleSet()->normalize();
    //filter.getSampleSet()->resample();
  }
} /* namespace ohmPf */
