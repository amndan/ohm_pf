/*
 * OdomUpdater.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ODOMUPDATER_H_
#define SRC_ODOMUPDATER_H_

#include "FilterUpdaterMeasurementOCS.h"
#include "Filter.h"
#include "OCSObserver.h"
#include "IOdomMeasurement.h"

namespace ohmPf
{

/**
 * @brief Abstract class to update the filter with a new odom measurement
 * if odom changed significantly (see @ref OCS) and actualize the
 * OCS-Observer with odom measurement updates.
 */
class OdomUpdater : public FilterUpdaterMeasurementOCS
{
public:
  /**
   * @brief Constructor
   * @param filter Pointer to the filter instance.
   * @param measurement Pointer to the odom measurement.
   * @param ocsObserver Pointer to the OCS observer.
   */
  OdomUpdater(Filter* filter, IOdomMeasurement* measurement, OCSObserver* ocsObserver, std::string idString);

  /**
   * @brief Destructor (empty)
   */
  virtual ~OdomUpdater(){};

  /**
   * @brief Try to update function is the trigger to update the filter. In odom
   * Updater we have to overwrite it for ocs management.
   */
  virtual bool tryToUpdate();

protected:
  /**
   * @brief This function gets called when there is a update requested.
   * Each OdomUpdater implementation must implement this method.
   */
  virtual void calculate() = 0;
  IOdomMeasurement* _odomMeasurement;

private:
  /**
   * @brief The update function for updating the filter with
   * the actual measurement using the calculate function.
   * Update just takes place, if odom has changed significantly (see @ref OCS)
   */
  void update();

  OCSObserver* _ocsObserver;

};

} /* namespace ohmPf */

#endif /* SRC_ODOMUPDATER_H_ */
