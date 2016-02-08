/*
 * Map.h
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include "Eigen/Dense"

namespace ohmPf
{
  class Map
  {
  public:
    Map(){}
    virtual ~Map(){}
    virtual bool isOccupied(double x, double y) = 0; //x, y in m
    virtual double getWith() = 0;
    virtual double getHeigh() = 0;
    virtual Eigen::Matrix3d getOrigin() = 0;
    virtual void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax) = 0;
  };
} /* namespace ohmPf */

#endif /* SRC_MAP_H_ */
