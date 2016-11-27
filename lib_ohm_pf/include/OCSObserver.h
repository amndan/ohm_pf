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

namespace ohmPf
{

/**
 * @brief OCS means Odom-Changed-Significantly. OCSObserver class observes odometry
 * data. Any OCSClient can register to OCSObserver with a specified odom dist x.
 * If incremental odom displacement overcomes dist x, the clients OCS-Flag is set
 * by OCSObserver.
 */
class OCSObserver
{
public:
  /**
   * @brief Constructor
   * @param rotToTransFactor Rotational and Translational dist has to be compared.
   * This is the factor for transforming rad to meter for this comparison.
   */
  OCSObserver(double rotToTransFactor);

  /**
   * Destructor (empty)
   */
  virtual ~OCSObserver(){};

  /**
   * @brief Add a client to the observer. If Odom changed more than dist
   * The Clients OCSFlag will be set to true.
   * @param client Pointer to the client.
   * @param dist Distance value after setting OCSFlag of Client to true.
   * The dist value itself has no unit because it is a mixture of rotational
   * and translation distances. If there are just Translational movements
   * it would be meters.
   */
  void registerClient(OCSClient* client, double dist);

  /**
   * @brief OCSObserver needs to know all odom measurements to manage OCSFlags.
   * @param measurement Pointer to the odom measurement to update with.
   */
  void update(IOdomMeasurement* measurement);

private:

  /**
   * @brief Calculates a distance value from rotational and translational displacement
   * between two poses m1 and m2
   * @param m1 pose 1: x y yaw in: m m rad
   * @param m2 pose 2: x y yaw in: m m rad
   * @return Dist valie (no unit)
   */
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
