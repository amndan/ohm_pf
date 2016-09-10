/*
 * LaserMeasurement.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "../include/ROSLaserMeasurement.h"

namespace ohmPf
{

ROSLaserMeasurement::ROSLaserMeasurement(const sensor_msgs::LaserScanConstPtr& scan,
                                         std::string tfBaseFootprintFrame,
                                         unsigned int subsamplingRate,
                                         double uncertainty)
{
  initWithMeasurement(scan,tfBaseFootprintFrame,subsamplingRate,uncertainty);
}

void ROSLaserMeasurement::initWithMeasurement(const sensor_msgs::LaserScanConstPtr& scan,
                                              std::string tfBaseFootprintFrame,
                                              unsigned int subsamplingRate,
                                              double uncertainty)
{
  _angleIncrement = (double)scan->angle_increment;
  _angleMax = (double)scan->angle_max;
  _angleMin = (double)scan->angle_min;
  _rangeMax = (double)scan->range_max;
  _rangeMin = (double)scan->range_min;

  if(uncertainty >= 0.0 && uncertainty < 1.0)
  {
    _uncertainty = uncertainty;
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "--> uncertainty >= 0.0 && uncertainty < 1.0: "
        "uncertainty = " << uncertainty << " --> exit()" << std::endl;
    exit(EXIT_FAILURE);
  }

  if(subsamplingRate > 1)
  {
    _subsamplingRate = subsamplingRate;
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << "--> subsampling rate of laser must be > 1. Subsampling under 2 is not"
        "yet implementet. Requestet subsampling factor is: " << subsamplingRate << std::endl;

    std::cout << __PRETTY_FUNCTION__ << "--> Will proceed with subsampling factor of 3" << std::endl;
    _subsamplingRate = 3;
  }

  std::string tfLaserFrame = scan->header.frame_id;

  double angleRange = std::abs(_angleMax - _angleMin);
  double angleIncrement = angleRange / (double) scan->ranges.size();

  if( abs(angleIncrement) > abs(_angleIncrement + 0.01) || abs(angleIncrement) < abs(_angleIncrement - 0.01))
  {
    ROS_WARN_STREAM("abs(angleIncrement) > abs(_angleIncrement + 0.01) || abs(angleIncrement) < abs(_angleIncrement - 0.01) "
        "abs(angleIncrement) = " << std::abs(angleIncrement) << "; abs(_angleIncrement) = " << std::abs(_angleIncrement));
  }

  _angleIncrement = angleIncrement;

  assert(_angleIncrement != 0);

  _count = scan->ranges.size();

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

  if(scan->ranges.size() != _count)
  {
    ROS_WARN_STREAM("scan->ranges.size() != _count " << scan->ranges.size() << " != " << _count);
  }
  else
  {
    _ranges = scan->ranges;
    _stamp = scan->header.stamp;
  }
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

unsigned int ROSLaserMeasurement::getSubsamplingRate()
{
  return _subsamplingRate;
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
