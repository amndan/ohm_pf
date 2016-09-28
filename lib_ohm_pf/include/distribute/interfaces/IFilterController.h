/*
 * IFilterController.h
 *
 *  Created on: Mar 13, 2016
 *      Author: amndan
 */

#ifndef SRC_IFILTERCONTROLLER_H_
#define SRC_IFILTERCONTROLLER_H_

#include "IOdomMeasurement.h"
#include "ILaserMeasurement.h"
#include "IFilterOutput.h"
#include "interfaces/IMap.h"
#include "FilterParams.h"
#include "OdomDiffParams.h"
#include "IPoseMeasurement.h"
#include "Eigen/Dense"

namespace ohmPf
{

/**
 * @brief Abstract class for providing access of the FilterController to the user.
 * This makes it possible to hide the filters implementation from the user.
 * This interface is not for implementing multible filter controllers.
 */
class IFilterController
{
public:
  /**
   * @brief Constructor (empty)
   */
  IFilterController(){};

  /**
   * Destructor (empty)
   */
  virtual ~IFilterController(){};

  /**
   * @brief static factory method. Implementation can be found in FilterController class.
   * The user can call this static function to create a FilterController object and to
   * initialize the Filter. The user has no information about the implementation of libOhmPf
   * and accesses all method trough this interface class.
   * @param params Filter parameter Struct
   * @return returns a pointer to the FilterController which can be accessed via this
   * IFilterController interface.
   */
  static IFilterController* createFilter(FilterParams_t params);

  /**
   * @brief Sets the filters Map instance.
   * @param map Map interface.
   * @return Returns true if successful.
   */
  virtual bool setMap(IMap* map) = 0;

  /**
   * @brief Sets the filters odom Measurement.
   * @param odom Odom measurement interface.
   * @param params Odometry params.
   * @return Returns true if successful.
   */
  virtual bool connectOdomMeasurement(IOdomMeasurement* odom, OdomParams_t params) = 0; // TODO: odom params and not odom diff params

  /**
   * @brief Sets the filters laser measurement.
   * @param laser Laser measurement interface.
   * @param laserId Laser id for multible laser scanners.
   * @return Returns true if successfu0l.
   */
  virtual bool connectLaserMeasurement(ILaserMeasurement* laser, unsigned int laserId) = 0;

  /**
   * @brief Sets the filters pose measurement interface.
   * @param pose pose measurement interface.
   * @return Returns true if successful.
   */
  virtual bool connectPoseMeasurement(IPoseMeasurement* pose) = 0;

  /**
   * @brief Sets the filters filter output interface.
   * @param output Output interface.
   * @return Returns true if successful.
   */
  virtual bool connectFilterOutput(IFilterOutput* output) = 0;

  /**
   * @brief Initializes the filter with its Map instance.
   * @return Returns true if successful.
   */
  virtual bool initFilterMap() = 0;

  /**
   * @brief Let the filter start to work.
   */
  virtual void filterSpinOnce() = 0;

  /**
   * @ Initializes the filter with a Pose in Map frame.
   * @param pose Pose for initializing the Filter.
   * @param sigTrans Translational standard deviation in m.
   * @param sigPhi Rotational standard deviation in rad.
   * @return Returns true if successful.
   */
  virtual bool initFilterPose(Eigen::Vector3d pose, double sigTrans, double sigPhi) = 0;

  virtual void requestProbMap(
      unsigned int& width,
      unsigned int& height,
      double resolution,
      Eigen::Matrix3d originTf,
      std::vector<int8_t>& mapData) = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IFILTERCONTROLLER_H_ */
