/*
 * OCSObserver.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: amndan
 */

#include "OCSObserver.h"

namespace ohmPf
{

OCSObserver::OCSObserver()
{
  _cumDist = 0;
  _initialized = false;
  _rotToTransFactor = 8; //TODO: magic number

}

OCSObserver::~OCSObserver()
{
  // TODO Auto-generated destructor stub
}

void OCSObserver::registerClient(IOCSClient* client, double dist)
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


    for(int i = 0; i < _clientList.size(); i++)
    {
      _actualDists.at(i) -= dist;

      if(_actualDists.at(i) < 0.0)
      {
        _clientList.at(i)->setOCSFlagTrue();
        _actualDists.at(i) = _dists.at(i);
      }
    }

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
