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
#include <cmath>

namespace ohmPf
{

  class CeilCam
  {
  public:
    void updateFilter(Filter* filter);
    void setMeasurement(std::vector<Eigen::Vector3d> measurement);
    CeilCam();
    virtual ~CeilCam();
  private:
    std::vector<Eigen::Vector3d> _measurement;
  };

} /* namespace ohmPf */

#endif /* SRC_CEILCAM_H_ */
