/*
 * LaserMeasurement.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "../include/ROSLaserMeasurement.h"

namespace ohmPf
{

ROSLaserMeasurement::ROSLaserMeasurement()
{
  _initialized = false;
}

ROSLaserMeasurement::~ROSLaserMeasurement()
{
  // TODO Auto-generated destructor stub
}

void ROSLaserMeasurement::initWithMeasurement(sensor_msgs::LaserScanConstPtr scan, std::string tfBaseFootprintFrame)
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

void ROSLaserMeasurement::setMeasurement(sensor_msgs::LaserScanConstPtr scan)
{
  assert(_initialized);
  //todo: set stamp here
  _ranges = scan->ranges;
  _stamp = scan->header.stamp;
}

ros::Time ROSLaserMeasurement::getStamp()
{
  return _stamp;
}

double ROSLaserMeasurement::getAngleIncrement()
{
  return _angleIncrement;
}

double ROSLaserMeasurement::getRangeMax()
{
  return _rangeMax;
}

double ROSLaserMeasurement::getRangeMin()
{
  return _rangeMin;
}

double ROSLaserMeasurement::getAngleMin()
{
  return _angleMin;
}

double ROSLaserMeasurement::getAngleMax()
{
  return _angleMax;
}

unsigned int ROSLaserMeasurement::getCount()
{
  return _count;
}

std::vector<float> ROSLaserMeasurement::getRanges()
{
  return _ranges;
}

double ROSLaserMeasurement::getUncertainty()
{
  return _uncertainty;
}

Eigen::Matrix3d ROSLaserMeasurement::getTfBaseFootprintToLaser()
{
  return _tfBaseFootprintToLaser;
}

bool ROSLaserMeasurement::isInitialized()
{
  return _initialized;
}

} /* namespace ohmPf */