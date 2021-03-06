/*
 * OCSObserver.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: amndan
 */

#include "OCSObserver.h"

namespace ohmPf
{

OCSObserver::OCSObserver(double rotToTransFactor)
{
  _cumDist = 0;
  _initialized = false;
  _rotToTransFactor = rotToTransFactor;
}

void OCSObserver::registerClient(OCSClient* client, double dist)
{
  assert(client != NULL);
  assert(dist >= 0);

  _clientList.push_back(client);
  _dists.push_back(dist);
  _actualDists.push_back(dist);

  std::cout << "registered new OCS client!" << std::endl;
}


void OCSObserver::update(IOdomMeasurement* measurement)
{
  if(!_initialized)
  {
    _lastOdomMeasurement = measurement->getMeasurement();
    _initialized = true;
    return;
  }
  else
  {
    double dist = calcDist(measurement->getMeasurement(), _lastOdomMeasurement);

    for(unsigned int i = 0; i < _clientList.size(); i++)
    {
      // decrement dist buffer
      _actualDists.at(i) -= dist;

      // if dist buffer under 0 odom changed sigificantly for this client
      if(_actualDists.at(i) < 0.0)
      {
        _clientList.at(i)->setOCSFlagTrue();
        _actualDists.at(i) = _dists.at(i);
      }
    }
    // remember new value
    _lastOdomMeasurement = measurement->getMeasurement();
  }
}

double OCSObserver::calcDist(Eigen::Vector3d m1, Eigen::Vector3d m2)
{
  Eigen::Vector3d diff;
  diff = m1 - m2;
  double dist = std::max(std::abs(diff(0)), std::abs(diff(1))); // no need for eucl dist
  dist += std::abs(diff(2)) * _rotToTransFactor;
  return dist;
}

} /* namespace ohmPf */
