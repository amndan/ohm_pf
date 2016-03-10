/*
 * IMap.h
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include "Eigen/Dense"
#include <ros/time.h>

namespace ohmPf
{
  class IMap
  {
  public:
    IMap(){}
    virtual ~IMap(){}
    virtual bool isOccupied(double x, double y) = 0; //x, y in m
    virtual double getProbability(double x, double y) = 0; // in meter
    virtual double getProbability(Eigen::Matrix3Xd& coords, double uncertainty) = 0; // todo: coords has to be constant
    virtual double getWith() = 0; // in meter
    virtual double getHeigh() = 0; // in meter
    virtual Eigen::Matrix3d getOrigin() = 0; // tf in meter
    virtual void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax) = 0; // in meter
    virtual void calcProbMap() = 0;
    virtual ros::Time getStamp() = 0;

  };
} /* namespace ohmPf */

#endif /* SRC_MAP_H_ */
