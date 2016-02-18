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
#include "MapModel.h"
#include "Filter.h"

#define IS_OCCUPIED_THRESHHOLD 50
#define IS_UNKNOWN_CELL -1

using namespace Eigen;

namespace ohmPf
{

  class RosMap : public MapModel
  {
  public:
    RosMap(const nav_msgs::OccupancyGrid& msg);
    virtual ~RosMap();
    bool isOccupied(double x, double y); // in meter
    double getProbability(double x, double y); // in meter
    double getProbability(Eigen::Matrix3Xd& coords);
    double getWith();
    double getHeigh();
    Eigen::Matrix3d getOrigin();
    void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax);
    void calcProbMap();
    void getProbMap(nav_msgs::OccupancyGrid& msg);
    void updateFilter(Filter& filter);
  private:
    std::vector<int8_t> _mapRaw;
    std::vector<int8_t> _probMap;
    //Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> _map;
    float _resolution;  // m/cell
    unsigned int _width; // cells
    unsigned int _height; // cells
    Eigen::Matrix3d _tfMapToMapOrigin;

    void PointInMapToOrigin(double& x, double& y); // in meter
    void PointInMapToOrigin(Eigen::Matrix3Xd& coords); // in meter
    int meterToCells(double x);
    bool isInMapRange(int x, int y);


  };

} /* namespace ohmPf */

#endif /* TESTING_ROSMAP_H_ */
