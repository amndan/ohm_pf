/*
 * RosMap.h
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#ifndef TESTING_ROSMAP_H_
#define TESTING_ROSMAP_H_

#include "nav_msgs/OccupancyGrid.h"
#include "ros/types.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Dense"
#include "assert.h"
#include "iostream"
#include "UtilitiesOhmPf.h"
#include <cmath>
#include "IMap.h"
#include <ros/time.h>

#define IS_OCCUPIED_THRESHHOLD 50
#define IS_UNKNOWN_CELL -1

using namespace Eigen;

namespace ohmPf
{

  class ROSMap : public IMap
  {
  public:
    ROSMap(const nav_msgs::OccupancyGrid& msg, unsigned int maxDistanceProbMap);
    virtual ~ROSMap();
    bool isOccupied(double x, double y); // in meter
    double getProbability(double x, double y); // in meter
    double getProbability(Eigen::Matrix3Xd& coords, double pRand);
    double getWith();
    double getHeigh();
    Eigen::Matrix3d getOrigin();
    void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax);
    void calcProbMap();
    void calcContourMap();
    void getProbMap(nav_msgs::OccupancyGrid& msg);
    ros::Time getStamp();
    Eigen::Matrix3d getTfMapToMapOrigin();

  private:
    std::vector<int8_t> _mapRaw;
    std::vector<int8_t> _probMap; 
    std::vector<int8_t> _contourMap;
    //Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> _map;
    float _resolution;  // m/cell
    unsigned int _width; // cells
    unsigned int _height; // cells
    Eigen::Matrix3d _tfMapToMapOrigin;

    void PointInMapToOrigin(double& x, double& y); // in meter
    void PointInMapToOrigin(Eigen::Matrix3Xd& coords); // in meter
    int meterToCells(double x);
    bool isInMapRange(int x, int y);

    unsigned int _maxDistanceProbMap; //TODO: in meter not in cells
    ros::Time _stamp;
  };

} /* namespace ohmPf */

#endif /* TESTING_ROSMAP_H_ */
