/*
 * IFilterOutput.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_IFILTEROUTPUT_H_
#define SRC_IFILTEROUTPUT_H_

#include "Sample.h"
#include <vector>
#include "Eigen/Dense"
#include "FilterState.h"

namespace ohmPf
{

  class IFilterOutput
  {
  public:
    IFilterOutput(){};
    virtual ~IFilterOutput(){};
    virtual void actualizeTF(Eigen::Vector3d pose) = 0;
    virtual void actualizeState(FilterState_t state) = 0;
    virtual void printSampleSet(std::vector<Sample_t>& samples) = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_IFILTEROUTPUT_H_ */
