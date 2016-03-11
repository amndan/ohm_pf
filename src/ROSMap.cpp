/*
 * RosMap.cpp
 *
 *  Created on: 10.01.2016
 *      Author: amndan
 */

#include "../include/ROSMap.h"

namespace ohmPf
{

  ROSMap::ROSMap(const nav_msgs::OccupancyGrid& msg, unsigned int maxDistanceProbMap)
  {
    // todo: clean variables
    _mapRaw = msg.data;
    _resolution = msg.info.resolution;
    _width = msg.info.width;  // width is in map_origin_system
    _height = msg.info.height;  // height is in map_origin_system
    _stamp = msg.header.stamp;
    _maxDistanceProbMap = maxDistanceProbMap;
    // todo: need getXmin() getXmax() functions for injecting particles over whole map

    tf::Transform tmp;
    tf::poseMsgToTF(msg.info.origin, tmp);
    _tfMapToMapOrigin = tfToEigenMatrix3x3(tmp);

    calcProbMap();  // todo: just do that if neccesary; handle getProb when no prob map is there

    //Eigen::Map< Madouble RosMap::getProbability(Eigen::Matrix3Xd coords)trix<int8_t, Eigen::Dynamic, Eigen::Dynamic> > mf(_mapRaw.data(), _width, _height);
    //_map = mf;
  }

  Eigen::Matrix3d ROSMap::getTfMapToMapOrigin()
  {
    return _tfMapToMapOrigin;
  }


  ROSMap::~ROSMap()
  {
    // TODO Auto-generated destructor stub
  }

  ros::Time ROSMap::getStamp()
  {
    return _stamp;
  }

  bool ROSMap::isOccupied(double x, double y)
  {
    PointInMapToOrigin(x, y);

    int x_c = meterToCells(x);
    int y_c = meterToCells(y);

    if(!isInMapRange(x_c, y_c))  // cause we are using the enclosing rect at initialisation there are values outside the map
    {
      return true;
    }

    if(_mapRaw[y_c * _width + x_c] > IS_OCCUPIED_THRESHHOLD || _mapRaw[y_c * _width + x_c] == -1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

//  void ROSMap::initFilter(Filter& filter)
//  {
//    double xMin;
//    double yMin;
//    double xMax;
//    double yMax;
//
//    getMinEnclRect(xMin, yMin, xMax, yMax);
//
//    // generate cloud
//    std::vector<Sample_t> samples;
//
//    for(unsigned int i = 0; i < filter.getParamSet()->samplesMax; i++)
//    {
//      Sample_t sample;
//      sample.weight = 1.0;
//      sample.pose(2) = drand48() * 2 * M_PI - M_PI;
//      // todo: more efficient way here
//      do
//      {
//        sample.pose(0) = drand48() * (xMax - xMin) + xMin;
//        sample.pose(1) = drand48() * (yMax - yMin) + yMin;
//      }
//      while(isOccupied(sample.pose(0), sample.pose(1)));
//      //todo: check if there is at least one field not occupied
//
//      samples.push_back(sample);
//    }
//
//    filter.setSamples(samples);
//  }

  bool ROSMap::isInMapRange(int x, int y)
  {
    if(x < 0 || y < 0 || x >= (int) _width || y >= (int) _height)
    {
      return false;
    }

    return true;
  }

  int ROSMap::meterToCells(double x)
  {
    assert(_resolution != 0);
    int ret = std::floor(x / _resolution);
    return ret;
  }

  double ROSMap::getProbability(Eigen::Matrix3Xd& coords, double pRand)
  {
    PointInMapToOrigin(coords);

    double prob;
    double probOfCoords = 1.0;
    int x;
    int y;

    for(unsigned int i = 0; i < coords.cols(); i++)  // whole scan
    {
      if(coords(2, i) != 0)
      {
        x = (int)std::floor(coords(0, i) / _resolution);
        y = (int)std::floor(coords(1, i) / _resolution);

        if(!isInMapRange(x, y))
        {
          prob = 0.0;
        }
        else
        {
          prob = (double)_probMap[y * _width + x] / 100.0;  // todo: coords wird hier verändert -> das darf nicht sein!!
        }
        prob = (1 - pRand) * prob + pRand;
        probOfCoords *= prob;
      }
    }
    return probOfCoords;
  }

  double ROSMap::getProbability(double x, double y)
  {
    PointInMapToOrigin(x, y);

    int x_c = meterToCells(x);
    int y_c = meterToCells(y);

    if(!isInMapRange(x_c, y_c))
    {
      return 0.0;
    }

    double prob = (double)_probMap[y_c * _width + x_c] / 100.0;

    assert(prob <= 1.0 && prob >= 0.0);

    return prob;
  }

  void ROSMap::PointInMapToOrigin(double& x, double& y)
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

//  void ROSMap::updateFilter(Filter& filter)
//  {
//    std::vector<Sample_t>* samples = filter.getSampleSet()->getSamples();
//
//    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
//    {
//      if(isOccupied(it->pose(0), it->pose(1)))
//      {
//        it->weight = 0.0;
//      }
//    }
//  }

  void ROSMap::PointInMapToOrigin(Eigen::Matrix3Xd& coords)
  {
    coords = _tfMapToMapOrigin.inverse() * coords;
    return;
  }

  double ROSMap::getHeigh()
  {
    return _height * _resolution;
  }

  double ROSMap::getWith()
  {
    return _width * _resolution;
  }

  Eigen::Matrix3d ROSMap::getOrigin()
  {
    return _tfMapToMapOrigin;
  }

  void ROSMap::getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax)
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

  void ROSMap::calcProbMap()
  {
    calcContourMap();
    
    int filterSize = std::ceil(1.0 * (double)_maxDistanceProbMap);  // in cells

    _probMap = _contourMap;
    double dist = 0.0;

    for(int i = 0; i < _width; i++)  // x map
    {
      for(int j = 0; j < _height; j++)  //y map
      {
        if(_contourMap[j * _width + i] > IS_OCCUPIED_THRESHHOLD)  // is occupied?
        {
          for(int k = -filterSize; k <= filterSize; k++)  // x filter
          {
            for(int l = -filterSize; l <= filterSize; l++)  //y filter
            {
              if((i + k) >= 0 && (i + k) < _width && (j + l) >= 0 && (j + l) < _height)
              {
                dist = std::sqrt(std::pow(k, 2) + std::pow(l, 2));  // precalculate that in future
                if(dist < _maxDistanceProbMap)
                {
                  dist = (1.0 - dist / _maxDistanceProbMap) * 100.0;  // normalize to [0;100]
                  _probMap[(j + l) * _width + (i + k)] = std::max((double)_probMap[(j + l) * _width + (i + k)], dist);
                }
              }
            }
          }
        }
        else if(_probMap[j * _width + i] == -1)
        {
          _probMap[j * _width + i] = 0;
        }
      }
    }
    std::cout << __PRETTY_FUNCTION__ << " --> created prob map!" << std::endl;
  }

  void ROSMap::calcContourMap()
  {
    int filterSize = 1;  // in cells

    _contourMap = _mapRaw;
    bool edge = false;

    for(int i = 0; i < _width; i++)  // x map
    {
      for(int j = 0; j < _height; j++)  //y map
      {
        if(_mapRaw[j * _width + i] == -1) continue;
        edge = false;
          for(int k = -filterSize; k <= filterSize; k++)  // x filter
          {
            for(int l = -filterSize; l <= filterSize; l++)  //y filter
            {
              if((i + k) >= 0 && (i + k) < _width && (j + l) >= 0 && (j + l) < _height)
              {
                edge = 
                  edge | 
                  (
                    (_mapRaw[j * _width + i] > IS_OCCUPIED_THRESHHOLD) ^
                    (_mapRaw[(j + l) * _width + (i + k)] > IS_OCCUPIED_THRESHHOLD)
                  );
              }
            }
          }
        _contourMap[j * _width + i] = 100 * (int8_t) edge;
      }
    }
    std::cout << __PRETTY_FUNCTION__ << " --> created contour map!" << std::endl;
  }


  void ROSMap::getProbMap(nav_msgs::OccupancyGrid& msg)
  {
    msg.data = _probMap;
  }

}
/* namespace ohmPf */
