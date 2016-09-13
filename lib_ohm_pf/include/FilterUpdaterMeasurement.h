/*
 * FilterUpdaterMeasurement.h
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENT_H_
#define LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENT_H_

#include "FilterUpdater.h"
#include "IMeasurement.h"
#include <iostream>
#include "ros/time.h"
#include "Filter.h"

namespace ohmPf
{

class FilterUpdaterMeasurement : public FilterUpdater
{
public:
  FilterUpdaterMeasurement(IMeasurement* measurement, Filter* filter);
  virtual ~FilterUpdaterMeasurement(){};

  virtual void tryToUpdate();

protected:
  ros::Time _lastStamp;
  IMeasurement* _measurement;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENT_H_ */
