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

/**
 * An abstract class for providing a generalized laser
 * measurement to the filter.
 */
class ILaserMeasurement : public IMeasurement
{
public:
  ILaserMeasurement(){};
  virtual ~ILaserMeasurement(){};

  /**
   * @brief Getter for angle increment.
   * @return angle increment in rad.
   */
  virtual double getAngleIncrement() = 0;

  /**
   * @brief Getter for max range.
   * @return max range in meter.
   */
  virtual double getRangeMax() = 0;

  /**
   * @brief Getter for min range.
   * @return min range in m.
   */
  virtual double getRangeMin() = 0;

  /**
   * @brief Getter for min angle.
   * @return min angle in rad.
   */
  virtual double getAngleMin() = 0;

  /**
   * @brief Getter for max angle.
   * @return max angle in rad.
   */
  virtual double getAngleMax() = 0;

  /**
   * @brief Getter for measurement count.
   * @return number of rays per scan.
   */
  virtual unsigned int getCount() = 0;

  /**
   * @brief Getter for actual measurement.
   * @return vector of ranges in m.
   */
  virtual std::vector<float> getRanges() = 0;

  /**
   * @brief Getter for subsampling rate.
   * @return subsampling rate. E.g. 3 means every 3rd scan is used.
   */
  virtual unsigned int getSubsamplingRate() = 0;

  /**
   * @brief Getter for additional uncertainty. This uncertainty
   * is used in LaserUpdater for adding an additional uncertainty
   * to hold the filter variance at a higher level. If The laser
   * measurement fits perfect and this uncertainty is e.g. 0.3
   * the weight of the particle is set to 0.3. This is kind of
   * mapping an overall uncertainty of the laser update method.
   * @todo It would be nice to keep also track of the maps uncertainty
   * to incorporate it into overall uncertainty of processes like
   * laser updates.
   * @return Additional uncertainty in range of [0.0;1.0[.
   */
  virtual double getUncertainty() = 0;

  /**
   * @brief Getter for tf from footprint of robot to laser frame.
   * @return tf with form: x y yaw in: m m rad
   */
  virtual Eigen::Matrix3d getTfBaseFootprintToLaser() = 0;


  /**
   * @brief Flag for initialization state.
   * @return If true, measurement is initialized.
   */
  virtual bool isInitialized() = 0;
};

} /* namespace ohmPf */

#endif /* SRC_LASERMEASUREMENT_H_ */
