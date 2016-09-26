/*
 * PoseUpdater.h
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#ifndef SRC_PoseUpdater_H_
#define SRC_PoseUpdater_H_

#include "FilterUpdaterMeasurementOCS.h"
#include "IPoseMeasurement.h"
#include "GaussianPdf.h"
#include "UtilitiesOhmPf.h"
#include "MapUpdater.h"

namespace ohmPf
{

/**
 * @brief An implementation of Filter Updater.
 * It processes pose measurements.
 */
class PoseUpdater : public FilterUpdaterMeasurementOCS
{
public:
  /**
   * @brief Constructor initializes PoseUpdater class.
   * @param filter pointer to The filter Instance to be updated.
   * @param measurement pointer to an IPoseMeasurement e.g. ROSCeilCamMeasurement.
   * The measurement itself gets actualized from outside this class.
   * After actualizing the measurement, update() should be called by the user.
   * @param updateFilterMap pointer to a map updater to trigger a map update for the filter
   * @todo Map update management isnt elegant --> how to trigger different updates when
   * separating updaters to different classes?
   * @todo PoseUpdater should be updated to universal interface for pose with covar arrays
   * also injecting or not injecting particles should be selectable
   */
  PoseUpdater(Filter* filter, IPoseMeasurement* measurement, MapUpdater* updateFilterMap, std::string idString);

  /**
   * Destructor (empty)
   */
  virtual ~PoseUpdater(){};

private:
  /**
   * @brief Update function for updating the filters particles with the actual PoseMeasurement.
   */
  void update();

  IPoseMeasurement* _poseMeasurement;
  MapUpdater* _updateFilterMap;
  void injectSamples();
};

} /* namespace ohmPf */

#endif /* SRC_PoseUpdater_H_ */
