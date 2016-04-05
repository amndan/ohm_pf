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
#include "IMap.h"
#include "FilterParams.h"
#include "OdomDiffParams.h"
#include "ICeilCamMeasurement.h"
#include "Eigen/Dense"

namespace ohmPf
{

class IFilterController
{
public:
  IFilterController(){};
  virtual ~IFilterController(){};

  static IFilterController* createFilter(FilterParams_t params);

  virtual bool setMap(IMap* map) = 0;
  virtual bool setOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params) = 0; // TODO: odom params and not odom diff params
  virtual bool setLaserMeasurement(ILaserMeasurement* laser) = 0;
  virtual bool setCeilCamMeasurement(ICeilCamMeasurement* ceilCam) = 0;
  virtual bool setFilterOutput(IFilterOutput* output) = 0;

  virtual bool updateLaser() = 0;
  virtual bool updateCeilCam() = 0;
  virtual bool updateOdom() = 0;
  virtual bool updateOutput() = 0;
  virtual bool resample() = 0;
  virtual bool initFilterMap() = 0;
  virtual bool initFilterPose(Eigen::Vector3d pose, double sigTrans, double sigPhi) = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IFILTERCONTROLLER_H_ */
