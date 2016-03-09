/*
 * ROSFilterOutput.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_ROSFILTEROUTPUT_H_
#define SRC_ROSFILTEROUTPUT_H_

#include "IFilterOutput.h"

namespace ohmPf
{

  class ROSFilterOutput : IFilterOutput
  {
  public:
    ROSFilterOutput();
    virtual ~ROSFilterOutput();
    void actualizeTF();
    void printSampleSet();
  };

} /* namespace ohmPf */

#endif /* SRC_ROSFILTEROUTPUT_H_ */
