/*
 * OdomDiff.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ODOMDIFF_H_
#define SRC_ODOMDIFF_H_

#include "IOdomQuantifier.h"
#include "assert.h"
#include "Filter.h"
#include "UtilitiesOhmPf.h"
#include "GaussianPdf.h"
#include "OdomDiffParams.h"
#include <cmath>

namespace ohmPf
{

  class OdomDiff : public IOdomQuantifier
  {
  public:
    OdomDiff(OdomDiffParams_t paramSet);
    virtual ~OdomDiff();
    void calculate(Filter& filter, IOdomMeasurement& measurement);
  private:
    void updatePose(Eigen::Vector3d& pose);
    void updateFilter(Filter& filter);
    void processMeasurement(Eigen::Vector3d odom);
    void calcParameters();

    Eigen::Vector3d _odom0;
    Eigen::Vector3d _odom1;
    double _dRot1;
    double _dTrans;
    double _dRot2;
    bool _initialized;
    bool _receivedFirstMeasurement;
    OdomDiffParams_t _paramSet;
  };

} /* namespace ohmPf */

#endif /* SRC_ODOMDIFF_H_ */
