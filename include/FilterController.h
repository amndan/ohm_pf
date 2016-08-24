/*
 * FilterController.h
 *
 *  Created on: Mar 13, 2016
 *      Author: amndan
 */

#ifndef SRC_FILTERCONTROLLER_H_
#define SRC_FILTERCONTROLLER_H_

#include "interfaces/IFilterController.h"
#include "IOdomMeasurement.h"
#include "ILaserMeasurement.h"
#include "interfaces/IFilterOutput.h"
#include "IMap.h"
#include "FilterParams.h"
#include "OdomUpdater.h"
#include "LaserUpdater.h"
#include "FilterOutputUpdater.h"
#include "interfaces/IResampler.h"
#include "MapUpdater.h"
#include "interfaces/IOdomQuantifier.h"
#include "OdomDiff.h"
#include "OdomDiffParams.h"
#include "assert.h"
#include "LaserProbMapMethod.h"
#include "LVResampler.h"
#include "STDResampler.h"
#include "FilterUpdater.h"
#include "OCSObserver.h"
#include "CeilCamUpdater.h"
#include "Eigen/Dense"
#include "LaserProbMapMethod.h"

namespace ohmPf
{

/**
 * @brief The Filter controller is the manager of the classes of ohmPf package.
 * He provides access to the hidden implementation of libOhmPf
 * via a its interface IFilterController. Via a factory method the user
 * creates the filter instance.
 */
class FilterController : public IFilterController
{
public:
  /**
   * @brief Constructor
   * @param params A filter params structure.
   */
  FilterController(FilterParams_t params);

  /**
   * @brief Destructor (empty)
   */
  virtual ~FilterController(){};

  /**
   * @brief Sets a reference to the filters map. The used map has to implement the
   * IMap interface. The filter needs a map to initialize itself.
   * @param map A pointer to a Map instance implementing the IMap interface.
   * @return returns true if map is successfully set.
   */
  bool setMap(IMap* map);

  /**
   * @brief Connect an odometry measurement to the Filter.
   * @param odom Pointer to an odometry measurement implementation implementing
   * IOdomMeasurement interface.
   * @param params Odometry parameter structure.
   * @todo Overload this function with e.g. OdomOmniParams for omnidirectional robots
   * @return returns true if odom measurement has been correctly connected
   */
  bool setOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params);

  /**
   * @brief Connect a single laser measurement to the filter
   * @param laser pointer to a laser measurement instance which is
   * implementing a ILaserMeasurement interface
   * @param laserId For multible laser setup user must give different laser
   * ids for different laser measurements.
   * @return returns true if laser measurement has been connected successfully
   */
  bool setLaserMeasurement(ILaserMeasurement* laser, unsigned int laserId = 0);

  bool setCeilCamMeasurement(ICeilCamMeasurement* ceilCam);
  bool setFilterOutput(IFilterOutput* output);

  bool updateLaser();
  bool updateLaser(unsigned int laserId);
  bool updateOdom();
  bool updateOutput();
  bool updateCeilCam();

  bool resample();

  bool initFilterMap();
  bool initFilterPose(Eigen::Vector3d pose, double sigTrans, double sigPhi);

private:
  IMap* _map;
  IOdomMeasurement* _odomMeasurement;
  std::vector<ILaserMeasurement*> _laserMeasurements;
  IFilterOutput* _filterOutput;
  FilterParams_t _filterParams;
  OdomUpdater* _odomUpdater;
  OCSObserver* _ocsObserver;
  std::vector<LaserUpdater*> _laserUpdaters;
  CeilCamUpdater* _ceilCamUpdater;
  FilterOutputUpdater* _outputUpdater;
  MapUpdater* _mapUpdater;
  IResampler* _resampler;
  Filter* _filter;
  LaserProbMapMethod* _laserQuantifier;
};

} /* namespace ohmPf */

#endif /* SRC_FILTERCONTROLLER_H_ */
