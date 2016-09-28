/*
 * OmniDriveUpdater.h
 *
 *  Created on: Sep 26, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_OMNIDRIVEUPDATER_H_
#define LIB_OHM_PF_SRC_OMNIDRIVEUPDATER_H_

#include "OdomUpdater.h"
#include "OdomDiffParams.h"
#include "Eigen/Dense"
#include <cmath>

namespace ohmPf
{

class OmniDriveUpdater : public OdomUpdater
{
public:
  OmniDriveUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver, OdomParams_t paramSet, std::string idString);
  virtual ~OmniDriveUpdater(){};

private:
  void calculate();
  void updatePose(Eigen::Vector3d& pose);
  void updateFilter(Filter& filter);
  void processMeasurement(Eigen::Vector3d odom);

  Eigen::Vector3d _odom0;
  Eigen::Vector3d _odom1;
  Eigen::Vector3d _diff;
  OdomParams_t _params;
  bool _receivedFirstMeasurement;

};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_OMNIDRIVEUPDATER_H_ */
