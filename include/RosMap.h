/*
 * RosMap.h
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#include "nav_msgs/OccupancyGrid.h"
#include "ros/types.h"
#include "Map.h"
#include "Eigen/Dense"
#include "assert.h"
#include "iostream"

#ifndef TESTING_ROSMAP_H_
#define TESTING_ROSMAP_H_

using namespace Eigen;

namespace ohmPf
{

  class RosMap : public Map
  {
  public:
    RosMap(const nav_msgs::OccupancyGrid& msg);
    virtual ~RosMap();
    bool isOccupied(double x, double y); // in meter
    double getWith();
    double getHeigh();
  private:
    std::vector<int8_t> _mapRaw;
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> _map;
    float _resolution;  // m/cell
    unsigned int _width; // cells
    unsigned int _height; // cells


  };

} /* namespace ohmPf */

#endif /* TESTING_ROSMAP_H_ */
