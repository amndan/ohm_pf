/*
 * CeilCam.h
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#ifndef SRC_CEILCAM_H_
#define SRC_CEILCAM_H_

#include "FilterUpdater.h"
#include "ICeilCamMeasurement.h"
#include "IUpdateFilterMap.h"
#include "GaussianPdf.h"
#include "UtilitiesOhmPf.h"

namespace ohmPf
{

  class CeilCamUpdater : public FilterUpdater
  {
  public:
    CeilCamUpdater(IFilter* filter, ICeilCamMeasurement* measurement, IUpdateFilterMap* updateFilterMap);
    virtual ~CeilCamUpdater();
    void update();
  private:
    ICeilCamMeasurement* _measurement;
    IUpdateFilterMap* _updateFilterMap;
  };

} /* namespace ohmPf */

#endif /* SRC_CEILCAM_H_ */
