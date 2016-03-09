/*
 * IFilter.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_IFILTER_H_
#define SRC_IFILTER_H_

#include <vector>

namespace ohmPf
{

  class IFilter
  {
  public:
    IFilter(){};
    virtual ~IFilter(){};
    virtual std::vector<Sample_t>* getSamples() = 0;
    virtual void setSamples(std::vector<Sample_t> samples) = 0;
  };

} /* namespace ohmPf */


#endif /* SRC_IFILTER_H_ */
