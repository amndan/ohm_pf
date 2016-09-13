/*
 * RosLaserPM.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LASERUPDATER_H_
#define INCLUDE_LASERUPDATER_H_

#include "FilterUpdaterMeasurementOCS.h"
#include "interfaces/IMap.h"
#include "ILaserMeasurement.h"
#include "MapUpdater.h"


namespace ohmPf
{

/**
 * @brief An abstract Class for updating the filter with a laser measurement.
 */
class LaserUpdater : public FilterUpdaterMeasurementOCS
{
public:

  /**
   * @brief Constructor initializes the laser updater class.
   * @param filter Pointer to the filter instance to update with the measurement.
   * @param map Pointer to a map instance.
   * @param measurement Pointer to the measurements representation.
   * @param updateFilterMap Pointer to the map updater instance.
   * @todo Perhaps implement a filterGetMap() beside the filterGetMapUpdater function.
   * MapUpdater and Map should belong to the filter. If the filter is initialized with
   * the map we can initialize all other updaters and give them a filter ref.
   */
  LaserUpdater(Filter* filter,
               IMap* map,
               ILaserMeasurement* measurement,
               MapUpdater* updateFilterMap);

  /**
   * Destructor (empty)
   */
  virtual ~LaserUpdater(){};



  /**
   * @brief This function gets called when there is a update requested.
   * Each LaserUpdater implementation must implement this method.
   */
  virtual void calculate() = 0;

protected:
  IMap* _map;
  ILaserMeasurement* _laserMeasurement;
  MapUpdater* _updateFilterMap;

private:
  /**
   * @brief The update function for updating the filter with
   * the actual measurement using the calculate function.
   * Update just takes place, if odom has changed significantly (see @ref OCS)
   */
  void update();


};

} /* namespace ohmPf */

#endif /* INCLUDE_LASERUPDATER_H_ */
