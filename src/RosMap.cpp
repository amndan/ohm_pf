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

    calcProbMap(); // todo: just do that if neccesary

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

    x = std::floor(x / _resolution);
    y = std::floor(y / _resolution);

    if (x < 0 || y < 0 || x > _width || y > _height)
    {
      return true;
    }

    if(_mapRaw[y * _width + x] > IS_OCCUPIED_THRESHHOLD)
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
    int maxDistance = 10; // in cells
    int filterSize = std::ceil(1.0 * (double) maxDistance); // in cells

    _probMap = _mapRaw;
    double dist = 0.0;

    for(int i = 0; i < _width; i++) // x
    {
      for(int j = 0; j < _height; j++)  //y
      {
        if(_mapRaw[j * _width + i] > IS_OCCUPIED_THRESHHOLD) // is occupied?
        {
          for(int k = -filterSize; k <= filterSize; k++) // x
          {
            for(int l = -filterSize; l <= filterSize; l++)  //y
            {
              if( (i + k) >= 0 && (i + k) < _width && (j + l) >= 0 && (j + l) < _height)
              {
                dist = std::sqrt(pow(k,2)+pow(l,2));
                if(dist < maxDistance)
                {
                  dist = (1.0 - dist / maxDistance) * 100.0; // normalize to [0;100]
                  _probMap[(j+l) * _width + (i+k)] = std::max((double)_probMap[(j+l) * _width + (i+k)], dist);
                }
              }
            }
          }
        }
      }
    }
    std::cout << __PRETTY_FUNCTION__ << " --> created prob map!" << std::endl;
  }

  void RosMap::getProbMap(nav_msgs::OccupancyGrid& msg)
  {
    msg.data = _probMap;
  }

}
/* namespace ohmPf */
