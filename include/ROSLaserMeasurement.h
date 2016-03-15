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
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "assert.h"
#include <ros/time.h>
#include "ILaserMeasurement.h"

namespace ohmPf
{

  class ROSLaserMeasurement : public ILaserMeasurement
  {
  public:
    ROSLaserMeasurement(double uncertainty);
    virtual ~ROSLaserMeasurement();
    void initWithMeasurement(const sensor_msgs::LaserScanConstPtr& scan, std::string tfBaseFootprintFrame);
    void setMeasurement(sensor_msgs::LaserScanConstPtr scan);
    ros::Time getStamp();
    double getAngleIncrement();
    double getRangeMax();
    double getRangeMin();
    double getAngleMin();
    double getAngleMax();
    unsigned int getCount();
    std::vector<float> getRanges();
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
