/*
 * OmniDriveUpdater.cpp
 *
 *  Created on: Sep 26, 2016
 *      Author: amndan
 */

#include "OmniDriveUpdater.h"

namespace ohmPf
{


OmniDriveUpdater::OmniDriveUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver, OdomParams_t paramSet, std::string idString) :
    OdomUpdater(filter, measurement, ocsObserver, idString)
{
  _diff(0) = 0;
  _diff(1) = 0;
  _diff(2) = 0;
  _receivedFirstMeasurement = false;
  _params = paramSet;

#if BENCHMARKING == 1
  std::cout << "warning: BENCHMARKING behavior is activated!" << std::endl;
  // for benchmarking
  deactivateOCS();
  _odom1(0) = 1.19953720228;
  _odom1(1) = 0.0;
  _odom1(2) = 0.0;
  _receivedFirstMeasurement = true;
  //~for benchmarking
#endif

}

void OmniDriveUpdater::calculate()
{
  if(!_receivedFirstMeasurement)
  {
    _odom1 = _odomMeasurement->getMeasurement();
    _receivedFirstMeasurement = true;
    return;
  }
  else
  {
    processMeasurement(_odomMeasurement->getMeasurement());
    updateFilter(*_filter);
    return;
  }
}

void OmniDriveUpdater::updatePose(Eigen::Vector3d& pose)
{
  Eigen::Vector3d tmp;

  // calculate diff
  _diff = _odom1 - _odom0;
  correctAngleOverflow(_diff(2));

  //turn movement vector in Odom0 frame
  tmp = _diff;
  _diff(0) = tmp(0) * std::cos((double)-_odom0(2)) - tmp(1) * std::sin((double)-_odom0(2));
  _diff(1) = tmp(0) * std::sin((double)-_odom0(2)) + tmp(1) * std::cos((double)-_odom0(2));

  // add noise trans
  tmp = _diff;
  _diff(0) +=  GaussianPdf::getRandomValue(0.0, std::abs(_params.a1 * tmp(0))); // x
  _diff(1) +=  GaussianPdf::getRandomValue(0.0, std::abs(_params.a2 * tmp(1))); // y

  double angleOdomToSample = pose(2);

  // turn movement vector to particle frame
  tmp = _diff;
  _diff(0) = tmp(0) * std::cos(angleOdomToSample) - tmp(1) * std::sin(angleOdomToSample);
  _diff(1) = tmp(0) * std::sin(angleOdomToSample) + tmp(1) * std::cos(angleOdomToSample);

  // add movement
  pose(0) += _diff(0);
  pose(1) += _diff(1);
  pose(2) += _diff(2) + GaussianPdf::getRandomValue(0.0, std::abs(_params.a3 * _diff(2))); // yaw;
  correctAngleOverflow(pose(2));
}

void OmniDriveUpdater::updateFilter(Filter& filter)
{
  std::vector<Sample_t>* samples = filter.getSamples();

  for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
  {
    updatePose(it->pose);
  }
}

void OmniDriveUpdater::processMeasurement(Eigen::Vector3d odom)
{
  _odom0 = _odom1;
  _odom1 = odom;
}



} /* namespace ohmPf */
































