/*
 * OCSObserver.h
 *
 *  Created on: Mar 18, 2016
 *      Author: amndan
 */

#ifndef SRC_OCSOBSERVER_H_
#define SRC_OCSOBSERVER_H_

#include "interfaces/IOdomMeasurement.h"
#include <vector>
#include "assert.h"
#include <cmath>
#include "OCSClient.h"

/**
 * @page OCS OCS-Management
 * Here comes the explanation of OCS-Management.
 * @todo add ocs management page.
 */

namespace ohmPf
{

/**
 * @brief Implementation of the OCS management.
 */
class OCSObserver
{
public:
  OCSObserver();
  virtual ~OCSObserver(){};
  void registerClient(OCSClient* client, double dist);
  void update(IOdomMeasurement* measurement);
private:

  double calcDist(Eigen::Vector3d m1, Eigen::Vector3d m2);
  std::vector<OCSClient*> _clientList;
  std::vector<double> _dists;
  std::vector<double> _actualDists;
  Eigen::Vector3d  _lastOdomMeasurement;
  double _cumDist;
  bool _initialized;
  double _rotToTransFactor;
};

} /* namespace ohmPf */

#endif /* SRC_OCSOBSERVER_H_ */
