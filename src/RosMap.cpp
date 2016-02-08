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
    _width = msg.info.width;  // width is in map_origin_system
    _height = msg.info.height;  // height is in map_origin_system
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

  bool RosMap::isOccupied(double x, double y)
  {
    PointInMapToOrigin(x, y);

    assert(x > 0 && y > 0);
    assert(x / _resolution < _width);  // todo: < | <=
    assert(y / _resolution < _height);

    x = std::floor(x / _resolution);
    y = std::floor(y / _resolution);

    if(_mapRaw[y * _width + x] == 0)
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

  Eigen::Matrix3d RosMap::getOrigin()
  {
    return _tfMapToMapOrigin;
  }

  void RosMap::getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax)
  {
    Eigen::MatrixXd rectInOrigin(3, 4);
    rectInOrigin.col(0) << 0, 0, 1;
    rectInOrigin.col(1) << getWith(), 0, 1;
    rectInOrigin.col(2) << 0, getHeigh(), 1;
    rectInOrigin.col(3) << getWith(), getHeigh(), 1;

    rectInOrigin = _tfMapToMapOrigin * rectInOrigin;

    xMax = rectInOrigin.row(0).maxCoeff();
    xMin = rectInOrigin.row(0).minCoeff();
    yMax = rectInOrigin.row(1).maxCoeff();
    yMin = rectInOrigin.row(1).minCoeff();
  }

  void RosMap::calcProbMap()
  {
//    unsigned int maxDistance = 5;
//    unsigned int filterSize = std::ceil(1.0 * (double) maxDistance);
//
//    _probMap = _mapRaw;
//
//    for(unsigned int i = 0; i < _width; i++) // x
//      for(unsigned int j = 0; j < _height; j++)  //y
//      {
//        if(_mapRaw[j * _width + i] == 0) // is occupied?
//        {
//          for(unsigned int k = -filterSize; i < _width; k++) // x
//            for(unsigned int l = 0; j < _height; l++)  //y
//        }
//      }
  }

}
/* namespace ohmPf */
