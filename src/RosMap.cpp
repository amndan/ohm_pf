/*
 * RosMap.cpp
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#include "RosMap.h"

namespace ohmPf
{

  RosMap::RosMap(const nav_msgs::OccupancyGrid& msg)
  {
    // todo: clean variables
    _mapRaw = msg.data;
    _resolution = msg.info.resolution;
    _width = msg.info.width; // width is in map_origin_system
    _height = msg.info.height; // height is in map_origin_system
    // todo: need getXmin() getXmax() functions for injecting particles over whole map

    tf::Transform tmp;
    tf::poseMsgToTF(msg.info.origin, tmp);
    _tfMapToMapOrigin = tfToEigenMatrix3x3(tmp);

    //Eigen::Map< Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> > mf(_mapRaw.data(), _width, _height);
    //_map = mf;
  }

  RosMap::~RosMap()
  {
    // TODO Auto-generated destructor stub
  }

  bool RosMap::isOccupied(double x, double y, bool isInMapOriginFrame)
  {
    if (!isInMapOriginFrame) PointInMapToOrigin(x, y);

    assert(x > 0 && y > 0);
    assert(x / _resolution  < _width); // todo: < | <=
    assert(y / _resolution < _height);

    x = std::floor(x / _resolution);
    y = std::floor(y / _resolution);

    if ( _mapRaw[y * _width + x] == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void RosMap::PointInMapToOrigin(double& x, double& y)
  {
    Eigen::Vector3d point;
    point(0) = x;
    point(1) = y;
    point(2) = 1;

    point = _tfMapToMapOrigin.inverse() * point;

    x = point(0);
    y = point(1);

    return;
  }

  double RosMap::getHeigh()
  {
    return _height * _resolution;
  }

  double RosMap::getWith()
  {
    return _width * _resolution;
  }


} /* namespace ohmPf */
