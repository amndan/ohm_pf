/*
 * RosLaserPM.h
 *
 *  Created on: 10.02.2016
 *      Author: amndan
 */

#ifndef INCLUDE_LASERUPDATER_H_
#define INCLUDE_LASERUPDATER_H_

#include "FilterUpdater.h"
#include "OCSClient.h"
#include "IMap.h"
#include "ILaserMeasurement.h"
#include "ILaserQuantifier.h"
#include "MapUpdater.h"


namespace ohmPf
{

/**
 * @brief A Class for updating the filter with a laser measurement.
 */
class LaserUpdater : public FilterUpdater, public OCSClient
{
public:

  /**
   * @brief Constructor initializes the laser updater class.
   * @param filter Pointer to the filter instance to update with the measurement.
   * @param map Pointer to a map instance.
   * @param measurement Pointer to the measurements representation.
   * @param quantifier Pointer to the laser quantifier to be used for update.
   * @param updateFilterMap Pointer to the map updater instance.
   * @todo Perhaps implement a filterGetMap() beside the filterGetMapUpdater function.
   * MapUpdater and Map should belong to the filter. If the filter is initialized with
   * the map we can initialize all other updaters and give them a filter ref.
   */
  LaserUpdater(Filter* filter,
               IMap* map,
               ILaserMeasurement* measurement,
               ILaserQuantifier* quantifier,
               MapUpdater* updateFilterMap);

  /**
   * Destructor (empty)
   */
  virtual ~LaserUpdater(){};

  /**
   * @brief The update function for updating the filter with the actual measurement using the quantifier.
   * Update just takes place, if odom has changed significantly (see @ref OCS)
   */
  void update();

private:
  IMap* _map;
  ILaserMeasurement* _measurement;
  ILaserQuantifier* _quantifier;
  MapUpdater* _updateFilterMap;
};

} /* namespace ohmPf */

#endif /* INCLUDE_LASERUPDATER_H_ */
