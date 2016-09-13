/*
 * FilterUpdaterTimed.h
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_FILTERUPDATERTIMED_H_
#define LIB_OHM_PF_SRC_FILTERUPDATERTIMED_H_

#include "FilterUpdater.h"
#include "ros/time.h"

namespace ohmPf
{

class FilterUpdaterTimed : public FilterUpdater
{
public:
  FilterUpdaterTimed(Filter* filter, ros::Duration intervall);

  virtual ~FilterUpdaterTimed(){};

  void tryToUpdate();

private:
  ros::Duration _intervall;
  ros::Time _lastStamp;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_FILTERUPDATERTIMED_H_ */
