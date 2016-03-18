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
#include "FilterUpdater.h"
#include "OCSObserver.h"
#include "CeilCamUpdater.h"

namespace ohmPf
{

class FilterController : public IFilterController
{
public:
  FilterController(FilterParams_t params);
  virtual ~FilterController();

  bool setMap(IMap* map);
  bool setOdomMeasurement(IOdomMeasurement* odom, OdomDiffParams_t params);
  bool setLaserMeasurement(ILaserMeasurement* laser);
  bool setCeilCamMeasurement(ICeilCamMeasurement* ceilCam);
  bool setFilterOutput(IFilterOutput* output);

  bool updateLaser();
  bool updateOdom();
  bool updateOutput();
  bool resample();
  bool updateCeilCam();
  bool initFilterMap();

private:
  IMap* _map;
  IOdomMeasurement* _odomMeasurement;
  ILaserMeasurement* _laserMeasurement;
  IFilterOutput* _filterOutput;

  OdomUpdater* _odomUpdater;
  OCSObserver* _ocsObserver;
  LaserUpdater* _laserUpdater;
  CeilCamUpdater* _ceilCamUpdater;
  FilterOutputUpdater* _outputUpdater;
  MapUpdater* _mapUpdater;
  IResampler* _resampler;
  Filter* _filter;
};

} /* namespace ohmPf */

#endif /* SRC_FILTERCONTROLLER_H_ */
