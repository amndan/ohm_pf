/*
 * ROSMap.h
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#ifndef SRC_ROSMAP_H_
#define SRC_ROSMAP_H_

#include "interfaces/IMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/types.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "iostream"
#include "UtilitiesOhmPf.h"
#include <cmath>
#include <ros/time.h>

#define IS_OCCUPIED_THRESHHOLD 50
#define IS_UNKNOWN_CELL -1

namespace ohmPf
{

/**
 * @brief An implementation of the IMap interface
 * for connection a ROS map to the filter. For detailed
 * description of implementation of abstract members
 * please see IMap.
 */
class ROSMap : public IMap
{
public:
  /**
   * @brief Constructor initializes with a ros map.
   * @param msg A ros map.
   */
  ROSMap(const nav_msgs::OccupancyGrid& msg);

  /**
   * Destructor (empty)
   */
  virtual ~ROSMap(){};

  /**
   * IMPLEMENTATION OF ABSTRACT MEMBERS
   */
  unsigned int isOccupied(int x, int y);  //x, y in cells
  unsigned int getHeighInCells();
  unsigned int getWidthInCells();
  double getResolution();
  Eigen::Matrix3d getTfMapToMapOrigin();
  std::vector<int8_t> getMapData();

private:
  std::vector<int8_t> _mapRaw;
  float _resolution;  // m/cell
  unsigned int _width; // cells
  unsigned int _height; // cells
  Eigen::Matrix3d _tfMapToMapOrigin;
  ros::Time _stamp;
};

} /* namespace ohmPf */

#endif /* SRC_ROSMAP_H_ */
