/*
 * DiffDriveUpdater.cpp
 *
 *  Created on: Sep 3, 2016
 *      Author: amndan
 */

#include "DiffDriveUpdater.h"

namespace ohmPf
{

DiffDriveUpdater::DiffDriveUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver, OdomParams_t paramSet, std::string idString) :
    OdomUpdater(filter, measurement, ocsObserver, idString)
{
  _dRot1 = 0.0;
  _dTrans = 0.0;
  _dRot2 = 0.0;
  _initialized = false;
  _receivedFirstMeasurement = false;
  _paramSet = paramSet;
}

void DiffDriveUpdater::calculate()
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
  }
}

void DiffDriveUpdater::updatePose(Eigen::Vector3d& pose)
{
  double sRot1;
  double sTrans;
  double sRot2;

  sRot1 = _dRot1 - GaussianPdf::getRandomValue(0.0, _paramSet.a1 * pow(_dRot1, 2) + _paramSet.a2 * pow(_dTrans, 2));
  sTrans = _dTrans - GaussianPdf::getRandomValue(0.0, _paramSet.a3 * pow(_dTrans, 2) +  //...
      _paramSet.a4 * pow(_dRot1, 2) + _paramSet.a4 * pow(_dRot2, 2));
  sRot2 = _dRot2 - GaussianPdf::getRandomValue(0.0, _paramSet.a1 * pow(_dRot2, 2) + _paramSet.a2 * pow(_dTrans, 2));

  pose(0) += sTrans * std::cos(pose(2) + sRot1);
  pose(1) += sTrans * std::sin(pose(2) + sRot1);
  pose(2) += sRot1 + sRot2;

  correctAngleOverflow(pose(2));

  //debug
  if(pose(2) < -2 * M_PI || pose(2) > 2 * M_PI)
    std::cout << __PRETTY_FUNCTION__ << "angle overflow; phi = " << pose(2) << std::endl;
}

void DiffDriveUpdater::updateFilter(Filter& filter)
{
  std::vector<Sample_t>* samples = filter.getSamples();

  for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
  {
    updatePose(it->pose);
  }
}

void DiffDriveUpdater::processMeasurement(Eigen::Vector3d odom)
{
  _odom0 = _odom1;
  _odom1 = odom;

  calcParameters();
  _initialized = true;
}

void DiffDriveUpdater::calcParameters()
{
  _dRot1 = std::atan2(_odom1(1) - _odom0(1), _odom1(0) - _odom0(0)) - _odom0(2);
  _dTrans = std::sqrt(pow(_odom1(0) - _odom0(0), 2) + pow(_odom1(1) - _odom0(1), 2));

  if(std::abs(_dRot1) > M_PI / 2)  // if more than 90 deg robot moves backwards
  {
    _dRot1 += M_PI;
    correctAngleOverflow(_dRot1);
    _dTrans *= -1;
  }

  _dRot2 = _odom1(2) - _odom0(2) - _dRot1;
}

} /* namespace ohmPf */
