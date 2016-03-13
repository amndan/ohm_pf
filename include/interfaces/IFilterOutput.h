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

namespace ohmPf
{

  class IFilterOutput
  {
  public:
    IFilterOutput(){};
    virtual ~IFilterOutput(){};
    virtual void actualizeTF() = 0;
    virtual void printSampleSet(std::vector<Sample_t>& samples) = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_IFILTEROUTPUT_H_ */
