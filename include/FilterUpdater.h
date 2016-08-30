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

/**
 * @brief Generic class for implementing filter updaters.
 * Every explicit filter updater should inherit from FilterUpdater.
 * This class ensures that every filter updater implements an update
 * function and has access to a filter via a private filter reference
 * @todo implement Time sync of filter updaters. _stamp is not used here.
 */
class FilterUpdater
{
public:
  /**
   * @brief Constructor: Needs filter pointer for initialization.
   * @param filter Pointer to the filter instance to be updated.
   */
  FilterUpdater(Filter* filter);

  /**
   * Destructor (empty)
   */
  virtual ~FilterUpdater(){};

  /**
   * Abstract update function. Every filter updater needs an update function.
   */
  virtual void update() = 0;


protected:
  /**
   * @brief Protected filter pointer for use in inheriting classes.
   */
  Filter* _filter;

};

} /* namespace ohmPf */

#endif /* SRC_FILTERUPDATER_H_ */
