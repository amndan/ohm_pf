/*
 * OCSObserver.h
 *
 *  Created on: Mar 18, 2016
 *      Author: amndan
 */

#ifndef SRC_OCSOBSERVER_H_
#define SRC_OCSOBSERVER_H_

#include "interfaces/IOCSClient.h"
#include "interfaces/IOdomMeasurement.h"
#include <vector>
#include "assert.h"
#include <cmath>

namespace ohmPf
{

class OCSObserver
{
public:
  OCSObserver();
  virtual ~OCSObserver();
  void registerClient(IOCSClient* client, double dist);
  void update(IOdomMeasurement* measurement);
private:

  double calcDist(Eigen::Vector3d m1, Eigen::Vector3d m2);
  std::vector<IOCSClient*> _clientList;
  std::vector<double> _dists;
  std::vector<double> _actualDists;
  Eigen::Vector3d  _lastOdomMeasurement;
  double _cumDist;
  bool _initialized;
  double _rotToTransFactor;
};

} /* namespace ohmPf */

#endif /* SRC_OCSOBSERVER_H_ */
