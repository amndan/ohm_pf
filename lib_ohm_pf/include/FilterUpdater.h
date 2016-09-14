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
   * @brief Destructor (empty)
   */
  virtual ~FilterUpdater(){};

  /**
   * @brief Try to update function is the trigger to update the filter. This function
   * can be overwritten by inheriting classes if they need to check e.g. stamps
   * before updating the filter.
   */
  virtual bool tryToUpdate();

protected:
  /**
   * @brief Abstract update function. Every filter updater needs an update function.
   * It is protected because FilterController just calls tryToUpdate() for updating
   * in case of everything is set for updating with this updater.
   */
  virtual void update() = 0;

  /**
   * @brief Protected filter pointer for use in inheriting classes.
   */
  Filter* _filter;
};

} /* namespace ohmPf */

#endif /* SRC_FILTERUPDATER_H_ */
