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

  CeilCam::~CeilCam()
  {
    // TODO Auto-generated destructor stub
  }

  void CeilCam::setMeasurement(std::vector<Eigen::Vector3d> measurement)
  {
    _measurement = measurement;
  }

  void CeilCam::updateFilter(Filter* filter)
  {
    // todo: better implementation
    std::vector<Sample_t>* samples = filter->getSampleSet()->getSamples();
    std::vector<double> dists(samples->size(), 10e10);

    for(std::vector<Eigen::Vector3d>::iterator it = _measurement.begin(); it != _measurement.end(); ++it)
    {
      double x = (*it)(0);
      double y = (*it)(1);
      double phi = (*it)(2);

      for(std::vector<Sample_t>::iterator it2 = samples->begin(); it2 != samples->end(); ++it2)
      {
        int n = std::distance(samples->begin(), it2);
        double tmpDist = std::sqrt( std::pow(it2->pose(0) - x,2) + //...
                  std::pow(it2->pose(1) - y,2)) +
                  std::pow(it2->pose(2) - phi,2);

        dists[n] = std::min(dists[n], tmpDist);
      }
    }

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      int n = std::distance(samples->begin(), it);
      it->weight = it->weight * GaussianPdf::getProbability(0.0, 2, dists[n]);
    }

  }

} /* namespace ohmPf */
