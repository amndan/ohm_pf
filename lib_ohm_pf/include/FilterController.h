/*
 * FilterController.h
 *
 *  Created on: Mar 13, 2016
 *      Author: amndan
 */

#ifndef SRC_FILTERCONTROLLER_H_
#define SRC_FILTERCONTROLLER_H_

#include "IFilterController.h"
#include "IOdomMeasurement.h"
#include "ILaserMeasurement.h"
#include "IFilterOutput.h"
#include "interfaces/IMap.h"
#include "FilterParams.h"
#include "DiffDriveUpdater.h"
#include "LaserUpdater.h"
#include "FilterOutputUpdater.h"
#include "IResampler.h"
#include "MapUpdater.h"
#include "OdomDiffParams.h"
#include "assert.h"
#include "LaserProbMapUpdater.h"
#include "LVResampler.h"
#include "STDResampler.h"
#include "ProbMap.h"
#include "OCSObserver.h"
#include "CeilCamUpdater.h"
#include "Eigen/Dense"

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
   * This could be e.g. ROSMap.
   * @return returns true if map is successfully set.
   */
  bool setMap(IMap* map);

  /**
   * @brief Connect an odometry measurement to the Filter.
   * @param odom Pointer to an odometry measurement implementation implementing
   * IOdomMeasurement interface e.g. ROSOdomMeasurement.
   * @param params Odometry parameter structure.
   * @todo Overload this function with e.g. OdomOmniParams for omnidirectional robots
   * @return returns true if odom measurement has been correctly connected
   */
  bool connectOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params);

  /**
   * @brief Connect a single laser measurement to the filter
   * @param laser pointer to a laser measurement instance which is
   * implementing a ILaserMeasurement interface e.g. ROSLaserMeasurement.
   * @param laserId For multible laser setup user must give different laser
   * ids for different laser measurements.
   * @return returns true if laser measurement has been connected successfully
   */
  bool connectLaserMeasurement(ILaserMeasurement* laser, unsigned int laserId = 0);

  /**
   * @brief connect a ceil cam measurement to the filter.
   * @param ceilCam Pointer to a ceil cam measurement object
   * which is implementing the ICeilCamMeasurement interface e.g. ROSCeilCamMeasurement.
   * @return return true if ceil cam measurement connected properly
   */
  bool connectCeilCamMeasurement(ICeilCamMeasurement* ceilCam);

  /**
   * @brief connect a filter output interface to the filter
   * @param output filter output mechanism which is implementing
   * a IFilterOutput interface e.g. ROSFilterOutput.
   * @return returns true if filter output has been connected successfully
   */
  bool connectFilterOutput(IFilterOutput* output);

  /**
   * @brief Updates the filter with the actual laser measurement
   * connected with setLaserMeasurement()
   * Updates the filter if OCS flag for measurement is set to true.
   * If not Function returns true but filter will not be updated.
   * @return returns true if filter is properly initialized to handle measurement
   */
  bool updateLaser(unsigned int laserId = 0);

  /**
   * @brief updates the filter with the actual odom measurement
   * connected with setOdomMeasurement()
   * Updates the filter if OCS flag for measurement is set to true.
   * If not Function returns true but filter will not be updated.
   * @return returns true if filter is properly initialized to handle measurement
   */
  bool updateOdom();

  /**
   * @brief updates the output object of the filter
   * which has been connected with setFilterOutput()
   * @return returns true if filter output was properly set
   */
  bool updateOutput();

  /**
   * @brief updates the filter with the actual ceil cam measurement
   * which has been connected with setCeilCamMeasurement()
   * @return returns true if ceil cam is sucessfully initialized
   */
  bool updateCeilCam();

  /**
   * @brief resamples the filter if OCS flag is active
   * @return returns true
   */
  bool resample();

  /**
   * @brief initializes/reinitializes the filters particles with its map data
   * @return returns true if map is available
   */
  bool initFilterMap();

  /**
   * @brief initializes/reinitializes the filters particles with a pose in map frame
   * @param pose pose in map frame to init filter
   * @param sigTrans translational standard deviation in m
   * @param sigPhi rotational standard deviation in rad
   * @return returns true if initialization was successfully
   */
  bool initFilterPose(Eigen::Vector3d pose, double sigTrans, double sigPhi);

  /**
   * @brief Function to request the probability map. So the user can display this map.
   * @param width Width of the map in cells.
   * @param height Height of the mao in cells.
   * @param resolution Resolution of the map in m/cells.
   * @param originTf The transformation from global map frame to the maps origin (cell (0|0)).
   * @param mapData Data array.
   * -1:        unknown
   * 0-100:     probability mapped from 0.0-1.0 to 0-100
   * @todo The data array is in ROS format. It should be a more general format definition.
   */
  void requestProbMap(
      unsigned int& width,
      unsigned int& height,
      double resolution,
      Eigen::Matrix3d originTf,
      std::vector<int8_t>& mapData);

private:
  IMap* _map;
  ProbMap* _probMap;
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
};

} /* namespace ohmPf */

#endif /* SRC_FILTERCONTROLLER_H_ */
