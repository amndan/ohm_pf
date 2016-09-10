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
    LaserProbMapMethod(double minValidRaysFactor);

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

    /**
     * @brief project laser range measurements into 2d-space relative to laser frame
     * @param measurement The laser measurement.
     * @return Coordinates array. Each point consists of three double values: x y z
     * x: the x value in m
     * y: the y value in m
     * z: used as validy mask; if z = 0 the coordinate is not valid because of invalid measurements from the laser.
     * @todo Implement subsampling of laser measurents with rates < 2.
     * @todo Implement min valid rays of scan routine.
     */
    Eigen::Matrix3Xd rangesToCoordinates(ILaserMeasurement& measurement);

    double _minValidRaysFactor;
  };
} /* namespace ohmPf */

#endif /* SRC_LASERPROBMAPMETHOD_H_ */
