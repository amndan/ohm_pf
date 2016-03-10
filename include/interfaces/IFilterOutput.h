/*
 * IFilterOutput.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_IFILTEROUTPUT_H_
#define SRC_IFILTEROUTPUT_H_

namespace ohmPf
{

  class IFilterOutput
  {
  public:
    IFilterOutput(){};
    virtual ~IFilterOutput(){};
    virtual void actualizeTF() = 0;
    virtual void printSampleSet() = 0;
  };

} /* namespace ohmPf */

#endif /* SRC_IFILTEROUTPUT_H_ */
