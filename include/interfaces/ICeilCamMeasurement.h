/*
 * ICeilCamMeasurement.h
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#ifndef SRC_ICEILCAMMEASUREMENT_H_
#define SRC_ICEILCAMMEASUREMENT_H_

#include "IMeasurement.h"
#include <vector>
#include "Eigen/Dense"

namespace ohmPf
{

  class ICeilCamMeasurement : public IMeasurement
  {
  public:
    ICeilCamMeasurement(){};
    virtual ~ICeilCamMeasurement(){};
    virtual std::vector<Eigen::Vector3d> getPoses() = 0;
    virtual std::vector<double> getProbabilities() = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_ICEILCAMMEASUREMENT_H_ */
