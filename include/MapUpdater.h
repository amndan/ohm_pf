/*
 * MapUpdater.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_MAPUPDATER_H_
#define SRC_MAPUPDATER_H_

#include "FilterUpdater.h"
#include "Filter.h"
#include "IMap.h"
#include "UtilitiesOhmPf.h"
#include "Eigen/Dense"

namespace ohmPf
{

/**
 * @brief Updater for updating/initializing a filters particles with information of a map.
 */
class MapUpdater : public FilterUpdater
{
public:
  /**
   * @brief Constructor
   * @param filter Pointer to the filter instance to be updated.
   * @param map Pointer to a map implementing the IMap interface, e.g. ROSMap.
   */
  MapUpdater(Filter* filter, IMap* map);

  /**
   * @brief Destructor (empty)
   */
  virtual ~MapUpdater(){};

  /**
   * @brief Init the filters particle with the information of the map.
   * @todo Implement random uncertainty in map update/init.
   * Few particles should survive on occupied places but not outside map bounds.
   * This requires more intelligent IMap::isOccupied() function.
   */
  void initFilter();

  /**
   * @brief Update the filters particles with the map information.
   * No resampling takes place here!
   */
  void update();

private:
  /**
   * @brief Calculates coordinates for generating random samples all over the map.
   * Map origin in map frame is variable, so before sampling for the whole map
   * x and y bounds must be known.
   * @param xMin minimum x bound in m.
   * @param yMin minimum y bound in m.
   * @param xMax maximum x bound in m.
   * @param yMax maximum y bound in m.
   */
  void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax);

  IMap* _map;
};

} /* namespace ohmPf */

#endif /* SRC_MAPUPDATER_H_ */
