/*
 * CeilCam.h
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#ifndef SRC_CEILCAM_H_
#define SRC_CEILCAM_H_

#include "FilterUpdater.h"
#include "ICeilCamMeasurement.h"
#include "GaussianPdf.h"
#include "UtilitiesOhmPf.h"
#include "MapUpdater.h"

namespace ohmPf
{

/**
 * @brief An implementation of Filter Updater.
 * It processes measurements from a ceiling cam.
 */
class CeilCamUpdater : public FilterUpdater
{
public:
  /**
   * @brief Constructor initializes CeilCamUpdater class.
   * @param filter pointer to The filter Instance to be updated.
   * @param measurement pointer to an ICeilCamMeasurement e.g. ROSCeilCamMeasurement.
   * The measurement itself gets actualized from outside this class.
   * After actualizing the measurement, update() should be called by the user.
   * @param updateFilterMap pointer to a map updater to trigger a map update for the filter
   * @todo Map update management isnt elegant --> how to trigger different updates when
   * separating updaters to different classes?
   * @todo CeilCamUpdater should be updated to universal interface for pose with covar arrays
   * also injecting or not injecting particles should be selectable
   */
  CeilCamUpdater(Filter* filter, ICeilCamMeasurement* measurement, MapUpdater* updateFilterMap);

  /**
   * Destructor (empty)
   */
  virtual ~CeilCamUpdater(){};

  /**
   * @brief Update function for updating the filters particles with the actual CeilCamMeasurement.
   */
  void update();

private:
  ICeilCamMeasurement* _measurement;
  MapUpdater* _updateFilterMap;
  void injectSamples();
};

} /* namespace ohmPf */

#endif /* SRC_CEILCAM_H_ */
