/*
 * RosMap.h
 *
 *  Created on: 13.01.2016
 *      Author: amndan
 */

#include "nav_msgs/OccupancyGrid.h"
#include "ros/types.h"
#include "tf/transform_datatypes.h"
#include "Map.h"
#include "Eigen/Dense"
#include "assert.h"
#include "iostream"
#include "UtilitiesOhmPf.h"

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
    Eigen::Matrix3d getOrigin();
    void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax);
    void calcProbMap();
  private:
    std::vector<int8_t> _mapRaw;
    std::vector<int8_t> _probMap;
    //Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> _map;
    float _resolution;  // m/cell
    unsigned int _width; // cells
    unsigned int _height; // cells
    Eigen::Matrix3d _tfMapToMapOrigin;

    void PointInMapToOrigin(double& x, double& y);


  };

} /* namespace ohmPf */

#endif /* TESTING_ROSMAP_H_ */
