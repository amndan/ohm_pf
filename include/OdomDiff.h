/*
 * OdomDiff.h
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#ifndef SRC_ODOMDIFF_H_
#define SRC_ODOMDIFF_H_

#include "OdomDiffParams.h"
#include "Sensor.h"
#include "Filter.h"
#include "Eigen/Dense"
#include "GaussianPdf.h"
#include <cmath>
#include <iostream>

#include "UtilitiesOhmPf.h"


namespace ohmPf
{

  class OdomDiff : public Sensor
  {
  public:
    OdomDiff(OdomDiffParams_t paramSet);
    virtual ~OdomDiff();
    void updateFilter(Filter& filter); // todo: integrate odom into filter; filter.setOdomModel; filter.updateWithOdom; odomModell.h --> abstract
    void initFilter(Filter& filter);
    void setMeasurement(Eigen::Vector3d odom0, Eigen::Vector3d odom1);
    void addSingleMeasurement(Eigen::Vector3d odom);
  private:
    void updatePose(Eigen::Vector3d& pose);
    void calcParameters();
    OdomDiffParams_t _paramSet;
    Eigen::Vector3d _odom0;
    Eigen::Vector3d _odom1;
    double _dRot1;
    double _dTrans;
    double _dRot2;
    bool _initialized;
    bool _receivedFirstMeasurement;
  };

} /* namespace ohmPf */

#endif /* SRC_ODOMDIFF_H_ */
