/*
 * FilterUpdater.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTERUPDATER_H_
#define SRC_FILTERUPDATER_H_

#include <ros/time.h>
#include "Filter.h"

namespace ohmPf
{

  class FilterUpdater
  {
  public:
    FilterUpdater(Filter* filter);
    virtual ~FilterUpdater(){};
    virtual void update() = 0;
    ros::Time getStamp();

  protected:
    Filter* _filter;
    ros::Time _stamp;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTERUPDATER_H_ */
