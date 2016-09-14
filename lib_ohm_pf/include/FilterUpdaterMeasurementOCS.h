/*
 * FilterUpdaterMeasurementOCS.h
 *
 *  Created on: Sep 13, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENTOCS_H_
#define LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENTOCS_H_

#include "FilterUpdaterMeasurement.h"
#include "OCSClient.h"

namespace ohmPf
{

/**
 * @brief An extension of FilterUpdater. It ensures that tryToUpdate
 * calls FilterUpdater::update() function only if a new measurement is
 * available AND OCS is triggered (see @ref OCS). New measurements are
 * detected by their time stamp.
 */
class FilterUpdaterMeasurementOCS : public FilterUpdaterMeasurement, public OCSClient
{
public:
  /**
   * @brief Constructor: Needs filter pointer for initialization.
   * @param filter Pointer to the filter instance to be updated.
   * @param measurement Measurement whose stamp gets compared.
   */
  FilterUpdaterMeasurementOCS(IMeasurement* measurement, Filter* filter);

  /**
   * @brief Destructor (empty)
   */
  virtual ~FilterUpdaterMeasurementOCS(){};

  /**
   * @brief FilterUpdaterMeasurementOCS overwrites tryToUpdate in a way that
   * the stamp of the measurement and OCS-State is checked before calling FilterUpdater::update().
   */
  virtual bool tryToUpdate();

protected:
  /**
   * @brief A switch for activating OCS management. (default: OCS = active)
   */
  void activateOCS();

  /**
   * @brief A switch for deactivating OCS management. (default: OCS = active)
   */
  void deactivateOCS();

private:
  bool _OCSactive;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_FILTERUPDATERMEASUREMENTOCS_H_ */
