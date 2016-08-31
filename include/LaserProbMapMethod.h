/*
 * LaserProbMapMethod.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_LASERPROBMAPMETHOD_H_
#define SRC_LASERPROBMAPMETHOD_H_

#include "ILaserQuantifier.h"
#include "IMap.h"
#include "ILaserMeasurement.h"
#include "Filter.h"
#include "UtilitiesOhmPf.h"
#include "Eigen/Dense"

namespace ohmPf
{
  class LaserProbMapMethod : public ILaserQuantifier
  {
  public:

    /**
     * @brief Constructor stores params.
     */
    LaserProbMapMethod(LaserProbMapParams_t params);

    /**
     * @brief Destructor (empty)
     */
    virtual ~LaserProbMapMethod(){};

    /**
     * @brief Implementation of the abstract calculate function. In this case calculation is
     * based on a probability map method.
     * @param filter Reference to the filter to be updated.
     * @param measurement Reference to the laser measurement.
     * @param map Reference to the map.
     * @param MapUpdater pointer to the filters map updater.
     */
    void calculate(Filter& filter, ILaserMeasurement& measurement, IMap& map, MapUpdater* updateFilterMap);
  private:

    Eigen::Matrix3Xd rangesToCoordinates(ILaserMeasurement& measurement);

    LaserProbMapParams_t _params;
  };
} /* namespace ohmPf */

#endif /* SRC_LASERPROBMAPMETHOD_H_ */
