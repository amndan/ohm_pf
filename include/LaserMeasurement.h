/*
 * LaserMeasurement.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_LASERMEASUREMENT_H_
#define SRC_LASERMEASUREMENT_H_

#include "Measurement.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
#include "UtilitiesOhmPf.h"
#include "tf/transform_datatypes.h"
#include "assert.h"

namespace ohmPf
{

  class LaserMeasurement : Measurement
  {
  public:
    LaserMeasurement();
    virtual ~LaserMeasurement();
    void initWithMeasurement(sensor_msgs::LaserScanConstPtr scan, std::string tfBaseFootprintFrame);
    void setMeasurement(sensor_msgs::LaserScanConstPtr scan);
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
  };

} /* namespace ohmPf */

#endif /* SRC_LASERMEASUREMENT_H_ */
