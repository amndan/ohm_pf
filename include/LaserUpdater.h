/*
 * RosLaserPM.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LASERUPDATER_H_
#define INCLUDE_LASERUPDATER_H_

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
#include "Measurement.h"
#include "UtilitiesOhmPf.h"

namespace ohmPf
{

  class LaserUpdater : public Measurement
  {
  public:
    LaserUpdater(RosLaserPMParams_t params);
    virtual ~LaserUpdater();
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

#endif /* INCLUDE_LASERUPDATER_H_ */
