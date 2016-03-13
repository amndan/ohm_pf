/*
 * OdomDiff.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "OdomDiff.h"

namespace ohmPf
{

  OdomDiff::OdomDiff(OdomDiffParams_t paramSet)
  {
    _initialized = false;
    _receivedFirstMeasurement = false;
    _paramSet = paramSet;
  }

  OdomDiff::~OdomDiff()
  {
    // TODO Auto-generated destructor stub
  }

  void OdomDiff::calculate(Filter& filter, IOdomMeasurement& measurement, IUpdateFilterMap& updateFilterMap)
  // TODO: updateFilterMap not used here; could use it in Future
  {
    if(!_receivedFirstMeasurement)
    {
      _odom1 = measurement.getMeasurement();
      _receivedFirstMeasurement = true;
      return;
    }
    else
    {
      processMeasurement(measurement.getMeasurement());
      updateFilter(filter);
    }
  }

  void OdomDiff::updatePose(Eigen::Vector3d& pose)
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

  void OdomDiff::updateFilter(Filter& filter)
  {
    assert(_initialized);

    std::vector<Sample_t>* samples = filter.getSamples();

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      updatePose(it->pose);
    }
  }

  void OdomDiff::processMeasurement(Eigen::Vector3d odom)
  {
    _odom0 = _odom1;
    _odom1 = odom;

    calcParameters();
    _initialized = true;
  }

  void OdomDiff::calcParameters()
  {
    _dRot1 = std::atan2(_odom1(1) - _odom0(1), _odom1(0) - _odom0(0)) - _odom0(2);
    _dTrans = std::sqrt(pow(_odom1(0) - _odom0(0), 2) + pow(_odom1(1) - _odom0(1), 2));
    _dRot2 = _odom1(2) - _odom0(2) - _dRot1;
  }

} /* namespace ohmPf */
