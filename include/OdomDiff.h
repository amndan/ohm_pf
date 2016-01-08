/*
 * OdomDiff.h
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#ifndef SRC_ODOMDIFF_H_
#define SRC_ODOMDIFF_H_

#include "OdomDiffParams.h"
#include "Filter.h"
#include "Eigen/Dense"
#include "GaussianPdf.h"
#include <cmath>
#include <iostream>


namespace ohmPf
{

  class OdomDiff
  {
  public:
    OdomDiff(OdomDiffParams_t paramSet);
    virtual ~OdomDiff();
    void updateFilter(Filter* filter);
    void setMeasurement(Eigen::Vector3d odom0, Eigen::Vector3d odom1);
  private:
    void updatePose(Eigen::Vector3d* pose);
    void calcParameters();
    OdomDiffParams_t _paramSet;
    Eigen::Vector3d _odom0;
    Eigen::Vector3d _odom1;
    double _dRot1;
    double _dTrans;
    double _dRot2;
    bool _initialized;
  };

} /* namespace ohmPf */

#endif /* SRC_ODOMDIFF_H_ */
