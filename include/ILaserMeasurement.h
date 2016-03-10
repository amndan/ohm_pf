/*
 * ILaserMeasurement.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_ILASERMEASUREMENT_H_
#define SRC_ILASERMEASUREMENT_H_

#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
#include "tf/transform_datatypes.h"
#include "IMeasurement.h"

namespace ohmPf
{

  class ILaserMeasurement : public IMeasurement
  {
  public:
    ILaserMeasurement(){};
    virtual ~ILaserMeasurement(){};
    virtual double getAngleIncrement() = 0;
    virtual double getRangeMax() = 0;
    virtual double getRangeMin() = 0;
    virtual double getAngleMin() = 0;
    virtual double getAngleMax() = 0;
    virtual unsigned int getCount() = 0;
    virtual std::vector<float> getRanges() = 0;
    virtual double getUncertainty() = 0;
    virtual Eigen::Matrix3d getTfBaseFootprintToLaser() = 0;
    virtual bool isInitialized() = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_LASERMEASUREMENT_H_ */
