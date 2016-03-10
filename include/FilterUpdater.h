/*
 * FilterUpdater.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTERUPDATER_H_
#define SRC_FILTERUPDATER_H_

#include "IFilter.h"
#include <ros/time.h>

namespace ohmPf
{

  class FilterUpdater
  {
  public:
    FilterUpdater(){};
    virtual ~FilterUpdater(){};
    virtual void update() = 0;
    ros::Time getStamp();

  private:
    IFilter* _filter;
    ros::Time _stamp;


  };

} /* namespace ohmPf */

#endif /* SRC_FILTERUPDATER_H_ */
