/*
 * LaserProbMapUpdater.h
 *
 *  Created on: Sep 3, 2016
 *      Author: amndan
 */

#ifndef SRC_LASERPROBMAPUPDATER_H_
#define SRC_LASERPROBMAPUPDATER_H_

#include "interfaces/IMap.h"
#include "ProbMap.h"
#include "ILaserMeasurement.h"
#include "MapUpdater.h"
#include "LaserUpdater.h"
#include "Filter.h"
#include "UtilitiesOhmPf.h"
#include "Eigen/Dense"
#include "Timer.h"
#include "AdaptiveMean.h"

namespace ohmPf
{

/**
 * @brief An implementation of a laser updater.
 * It actualizes the filters samples with taking the actual
 * laser measurement and map into account.
 */
class LaserProbMapUpdater: public LaserUpdater
{
public:

  /**
   * @brief Constructor initializes the LaserProbMapUpdater class.
   * @param filter Pointer to the filter instance to update with the measurement.
   * @param map Pointer to a map instance.
   * @param measurement Pointer to the measurements representation.
   * @param updateFilterMap Pointer to the map updater instance.
   * @param minValidRaysFactor The minimum percentage of valid scan rays to update.
   * 1.0 --> 100% --> each scan ray must be valid --> no out of range values, no inf/nan values
   * This Feature is not yet implemented!
   * @todo Implement minValidRaysFactor support.
   * @todo Implement subsampling of laser measurents with rates < 2.
   */
  LaserProbMapUpdater(Filter* filter, ProbMap* map, ILaserMeasurement* measurement, MapUpdater* updateFilterMap, double minValidRaysFactor, std::string idString, bool activateAdaptiveMean);

  /**
   * Destructor (empty)
   */
  virtual ~LaserProbMapUpdater(){};

  /**
   * @brief Actualize the filter with the scan and the probability map.
   */
  void calculate();

private:
  /**
   * @brief project laser range measurements into 2d-space relative to laser frame
   * @param measurement The laser measurement.
   * @return Coordinates array. Each point consists of three double values: x y z
   * x: the x value in m
   * y: the y value in m
   * z: used as validy mask; if z = 0 the coordinate is not valid because of invalid measurements from the laser.
   */
  Eigen::Matrix3Xd rangesToCoordinates(ILaserMeasurement& measurement);

  double _minValidRaysFactor;
  bool _adaptiveMeanIsActive;
  AdaptiveMean _adaptiveMean;
};

} /* namespace ohmPf */

#endif /* SRC_LASERPROBMAPUPDATER_H_ */
