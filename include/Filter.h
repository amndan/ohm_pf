/*
 * Filter.h
 *
 *  Created on: 08.01.2016
 *      Author: amndan
 */

#ifndef SRC_FILTER_H_
#define SRC_FILTER_H_

#include "SampleSet.h"
#include "Sample.h"
#include "FilterParams.h"
#include "Eigen/Dense"
#include "Sample.h"
#include "GaussianPdf.h"
#include <assert.h>
#include "ros/time.h" // ros time can be used without ros environment -- compile with "-lboost_system"
#include "ros/duration.h"
#include "MapModel.h"
#include "Sensor.h"
#include "EnumSensor.h"

namespace ohmPf
{

  class Filter
  {
  public:
    Filter(FilterParams_t paramSet);
    virtual ~Filter();
    SampleSet* getSampleSet();
    void setSamples(std::vector<Sample_t> samples);
    void initWithPose(const Eigen::Vector3d& pose);
    void initWithSensor(int sensorID);
    void updateWithSensor(int sensorID);
    bool isInitialized();
    FilterParams_t const * const getParamSet();
    Sensor& getSensor(int sensorID);
    void setSensor(int sensorID, Sensor* pSensor);
    void triggerOdomChangedSignificantly();


  private:
    SampleSet* _sampleSet;
    FilterParams_t _paramSet;
    bool _initialized;
    Sensor* _sensors[CNT_SENSORS];
    bool _sensorsInitialized[CNT_SENSORS];
    double _sensorsUpdateDistance[CNT_SENSORS]; // todo: every sensor should have its own significant odom change threshold
    bool _sensorsOdomChangedSignificantly[CNT_SENSORS];

  };

} /* namespace ohmPf */

#endif /* SRC_FILTER_H_ */
