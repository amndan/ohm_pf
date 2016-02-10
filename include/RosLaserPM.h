/*
 * RosLaserPM.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_ROSLASERPM_H_
#define INCLUDE_ROSLASERPM_H_

#include "LaserModel.h"
#include "Filter.h"
#include "sensor_msgs/LaserScan.h"
#include "RosLaserPMParams.h"
#include "assert.h"
#include "Eigen/Dense"
#include <cmath>
#include "UtilitiesOhmPf.h"
#include "Map.h"

namespace ohmPf
{

  class RosLaserPM : public LaserModel
  {
  public:
    RosLaserPM();
    virtual ~RosLaserPM();
    void updateFilter(Filter& filter);
    void setMeasurement(const sensor_msgs::LaserScanConstPtr& scanMsg);


  private:
    void initWithMeasurement();
    bool _initialized;
    RosLaserPMParams_t _paramSet;
    Eigen::Matrix3Xd rangesToCoordinates(std::vector<float>& ranges);
    sensor_msgs::LaserScan _actualScan;
  };

} /* namespace ohmPf */

#endif /* INCLUDE_ROSLASERPM_H_ */
