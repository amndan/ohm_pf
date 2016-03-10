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

  class IMeasurement
  {
  public:
    IMeasurement(){};
    virtual ~IMeasurement(){};
    virtual ros::Time getStamp() = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_MEASUREMENT_H_ */
