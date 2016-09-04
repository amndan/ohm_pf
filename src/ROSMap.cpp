/*
 * ROSMap.cpp
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#include "ROSMap.h"

namespace ohmPf
{

ROSMap::ROSMap(const nav_msgs::OccupancyGrid& msg)
{
  _mapRaw = msg.data;
  _resolution = msg.info.resolution;
  assert(_resolution != 0);
  _width = msg.info.width;  // width is in map_origin_system
  _height = msg.info.height;  // height is in map_origin_system
  _stamp = msg.header.stamp;
  tf::Transform tmp;
  tf::poseMsgToTF(msg.info.origin, tmp);
  _tfMapToMapOrigin = tfToEigenMatrix3x3(tmp);
}

std::vector<int8_t> ROSMap::getMapRaw()
{
  ROS_WARN("Warning ohmPf is using return _mapRaw workaround! ProbMap cannot rely on this output"
      " it should generate its own raw map data matrix");
  return _mapRaw;
}

unsigned int ROSMap::isOccupied(int x_c, int y_c)  //x, y in cells
{
  if(!isInMapRange(x_c, y_c))
  {
    return OCC_STATES::OUTBOUND;
  }

  int8_t value = _mapRaw[y_c * _width + x_c];

  if(value > IS_OCCUPIED_THRESHHOLD)
  {
    return OCC_STATES::OCCUPIED;
  }

  if(value == IS_UNKNOWN_CELL)
  {
    return OCC_STATES::UNKNOWN;
  }

  return OCC_STATES::FREE;
}

unsigned int ROSMap::getHeighInCells()
{
  return _height;
}

unsigned int ROSMap::getWidthInCells()
{
  return _width;
}

double ROSMap::getResolution()
{
  return _resolution;
}

Eigen::Matrix3d ROSMap::getTfMapToMapOrigin()
{
  return _tfMapToMapOrigin;
}

} /* namespace ohmPf */
