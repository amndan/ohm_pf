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

/**
 * @brief An extension of FilterUpdater. It ensures that tryToUpdate
 * calls FilterUpdater::update() function only if a certain time is elapsed.
 */
class FilterUpdaterTimed : public FilterUpdater
{
public:
  /**
   * @brief Constructor: Needs filter pointer for initialization.
   * @param filter Pointer to the filter instance to be updated.
   * @param time intervall for update
   */
  FilterUpdaterTimed(Filter* filter, ros::Duration intervall);

  /**
   * @brief Destructor (empty)
   */
  virtual ~FilterUpdaterTimed(){};

  /**
   * @brief FilterUpdaterTimed overwrites tryToUpdate in a way that
   * FilterUpdater::update() gets called if time period is elapsed.
   */
  virtual bool tryToUpdate();

private:
  ros::Duration _intervall;
  ros::Time _lastStamp;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_FILTERUPDATERTIMED_H_ */
