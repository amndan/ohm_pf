/*
 * CeilCam.h
 *
 *  Created on: 11.01.2016
 *      Author: amndan
 */

#ifndef SRC_CEILCAM_H_
#define SRC_CEILCAM_H_

#include "Sample.h"
#include "Filter.h"
#include "GaussianPdf.h"
#include "UtilitiesOhmPf.h"
#include <cmath>

#include "Measurement.h"

namespace ohmPf
{

  class CeilCamUpdater : public Measurement
  {
  public:
    void updateFilter(Filter& filter);
    void initFilter(Filter& filter);
    void setMeasurement(std::vector<Eigen::Vector3d> measurement);
    CeilCamUpdater();
    virtual ~CeilCamUpdater();
  private:
    std::vector<Eigen::Vector3d> _measurement;
  };

} /* namespace ohmPf */

#endif /* SRC_CEILCAM_H_ */
