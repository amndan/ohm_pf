/*
 * RosLaserPM.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_ROSLASERPM_H_
#define INCLUDE_ROSLASERPM_H_

#include "Sensor.h"
#include "Filter.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "RosLaserPMParams.h"
#include "assert.h"
#include "Eigen/Dense"
#include <cmath>
#include "EnumSensor.h"
#include "MapModel.h"
#include "UtilitiesOhmPf.h"

namespace ohmPf
{

  class RosLaserPM : public Sensor
  {
  public:
    RosLaserPM(RosLaserPMParams_t params);
    virtual ~RosLaserPM();
    void updateFilter(Filter& filter);
    void initFilter(Filter& filter);
    void setMeasurement(const sensor_msgs::LaserScanConstPtr& scanMsg);


  private:
    void initWithMeasurement();
    bool _initialized;
    RosLaserPMParams_t _paramSet;
    Eigen::Matrix3Xd rangesToCoordinates(std::vector<float>& ranges);
    sensor_msgs::LaserScan _actualScan;
    Eigen::Matrix3d _tfBaseFootprintToLaser;

  };

} /* namespace ohmPf */

#endif /* INCLUDE_ROSLASERPM_H_ */
