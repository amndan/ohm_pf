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

/**
 * @brief An extension of FilterUpdater. It ensures that tryToUpdate
 * calls FilterUpdater::update() function only if a new measurement is
 * available. New measurements are detected by their time stamp.
 * @todo implement Time sync of filter updaters. _stamp is not analysed here
 */
class FilterUpdaterMeasurement : public FilterUpdater
{
public:
  /**
   * @brief Constructor: Needs filter pointer for initialization.
   * @param filter Pointer to the filter instance to be updated.
   * @param measurement Measurement whose stamp gets compared.
   */
  FilterUpdaterMeasurement(IMeasurement* measurement, Filter* filter, std::string idString);

  /**
   * @brief Destructor (empty)
   */
  virtual ~FilterUpdaterMeasurement(){};

  /**
   * @brief FilterUpdaterMeasurement overwrites tryToUpdate in a way that
   * the stamp of the measurement gets compared to the last stamp before
   * calling FilterUpdater::update().
   */
  virtual bool tryToUpdate();

  IMeasurement* getMeasurement();

protected:
  ros::Time _lastStamp;
  IMeasurement* _measurement;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENT_H_ */
