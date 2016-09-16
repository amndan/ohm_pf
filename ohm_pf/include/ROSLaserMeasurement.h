/*
 * LaserMeasurement.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_ROSLASERMEASUREMENT_H_
#define SRC_ROSLASERMEASUREMENT_H_

#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
#include "UtilitiesOhmPf.h"
#include "UtilitiesOhmPfROS.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "assert.h"
#include <ros/time.h>
#include "ILaserMeasurement.h"

namespace ohmPf
{

/**
 * @brief An implementation of a laser measurement for the ros middleware.
 * For detailed member description please look at ILaserMeasurement.
 */
class ROSLaserMeasurement : public ILaserMeasurement
{
public:
  /**
   * @brief Constructor initializes the measurement.
   * @param scan ROS laser scan message.
   * @param tfBaseFootprintFrame Name of the base_footprint frame.
   * The name of the laser frame is available in the laser message.
   * @param subsamplingRate Subsampling rate to use. Subsampling rate
   * is just stored --> the scan does not get subsampled here.
   * @param uncertainty The overall laser uncertainty.
   * (see ILaserMeasurement::getUncertainty() )
   */
  ROSLaserMeasurement(const sensor_msgs::LaserScanConstPtr& scan,
                      std::string tfBaseFootprintFrame,
                      unsigned int subsamplingRate,
                      double uncertainty);

  /**
   * @brief Destructor (empty)
   */
  virtual ~ROSLaserMeasurement(){};

  /**
   * @brief Gets called from constructor with same parameters.
   */
  void initWithMeasurement(const sensor_msgs::LaserScanConstPtr& scan,
                           std::string tfBaseFootprintFrame,
                           unsigned int subsamplingRate,
                           double uncertainty);

  /**
   * @brief Sets the actual measurement data.
   * @param scan ROS laser scan message.
   */
  void setMeasurement(sensor_msgs::LaserScanConstPtr scan);

  ros::Time getStamp();
  double getAngleIncrement();
  double getRangeMax();
  double getRangeMin();
  double getAngleMin();
  double getAngleMax();
  unsigned int getCount();
  std::vector<float> getRanges();
  unsigned int getSubsamplingRate();
  double getUncertainty();
  Eigen::Matrix3d getTfBaseFootprintToLaser();
  bool isInitialized();

private:
  double _angleIncrement;
  double _rangeMax;
  double _rangeMin;
  double _angleMin;
  double _angleMax;
  unsigned int _count;
  unsigned int _subsamplingRate;
  double _uncertainty; // range: [0 1[
  Eigen::Matrix3d _tfBaseFootprintToLaser;
  bool _initialized;
  std::vector<float> _ranges;
  ros::Time _stamp;
};

} /* namespace ohmPf */

#endif /* SRC_LASERMEASUREMENT_H_ */
