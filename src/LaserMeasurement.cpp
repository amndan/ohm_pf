/*
 * LaserMeasurement.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "LaserMeasurement.h"

namespace ohmPf
{

  LaserMeasurement::LaserMeasurement()
  {
    _initialized = false;
  }

  LaserMeasurement::~LaserMeasurement()
  {
    // TODO Auto-generated destructor stub
  }

  void LaserMeasurement::initWithMeasurement(sensor_msgs::LaserScanConstPtr scan, std::string tfBaseFootprintFrame)
  {
    _angleIncrement = (double)scan->angle_increment;
    _angleMax = (double)scan->angle_max;
    _angleMin = (double)scan->angle_min;
    _rangeMax = (double)scan->range_max;
    _rangeMin = (double)scan->range_min;

    std::string tfLaserFrame = scan->header.frame_id;

    double angleRange = _angleMax - _angleMin;

    assert(angleRange > 0.0);
    assert(_angleIncrement != 0);

    int count = (int)(angleRange / _angleIncrement);  // todo: angle inc and ranges.size() dont equals!

    //assert(scan->ranges.size() == count);

    count = scan->ranges.size();
    _angleIncrement = std::abs(angleRange) / count;

    tf::Transform tf;
    tf::StampedTransform tmp;
    tf::TransformListener tfListener;

    tfListener.waitForTransform(tfBaseFootprintFrame, tfLaserFrame, ros::Time(0), ros::Duration(3.0));
    assert(tfListener.canTransform(tfBaseFootprintFrame, tfLaserFrame, ros::Time(0)));
    tfListener.lookupTransform(tfBaseFootprintFrame, tfLaserFrame, ros::Time(0), tmp);

    tf = tmp;  // stamped to not stamped

    _tfBaseFootprintToLaser = tfToEigenMatrix3x3(tf);

    _initialized = true;
  }

  void LaserMeasurement::setMeasurement(sensor_msgs::LaserScanConstPtr scan)
  {
    assert(_initialized);
    //todo: set stamp here
    _ranges = scan->ranges;
  }

} /* namespace ohmPf */
