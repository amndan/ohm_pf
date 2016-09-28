/*
 * DiffDriveUpdater.h
 *
 *  Created on: Sep 3, 2016
 *      Author: amndan
 */

#ifndef SRC_DIFFDRIVEUPDATER_H_
#define SRC_DIFFDRIVEUPDATER_H_

#include <OdomUpdater.h>
#include "assert.h"
#include "Filter.h"
#include "UtilitiesOhmPf.h"
#include "GaussianPdf.h"
#include "OdomDiffParams.h"
#include "OCSObserver.h"
#include "IOdomMeasurement.h"
#include <cmath>

namespace ohmPf
{

class DiffDriveUpdater : public OdomUpdater
{
public:
  DiffDriveUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver, OdomParams_t paramSet, std::string idString);
  virtual ~DiffDriveUpdater(){};

private:
  void calculate();
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
  OdomParams_t _paramSet;
};

} /* namespace ohmPf */

#endif /* SRC_DIFFDRIVEUPDATER_H_ */
