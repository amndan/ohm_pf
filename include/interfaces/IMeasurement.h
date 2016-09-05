/*
 * IMeasurement.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_IMEASUREMENT_H_
#define SRC_IMEASUREMENT_H_

#include <ros/time.h>

namespace ohmPf
{

/**
 * @brief An Interface for a generic measurement.
 */
class IMeasurement
{
public:
  IMeasurement(){};
  virtual ~IMeasurement(){};

  /**
   * @brief Each measurement should manage its actual time stamp.
   * @return time stamp of actual measurement.
   */
  virtual ros::Time getStamp() = 0; // TODO: should do this not virtual and relate to this class
};

} /* namespace ohmPf */

#endif /* SRC_MEASUREMENT_H_ */
