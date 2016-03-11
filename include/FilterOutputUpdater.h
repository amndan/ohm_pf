/*
 * FilterOutputUpdater.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTEROUTPUTUPDATER_H_
#define SRC_FILTEROUTPUTUPDATER_H_

#include "FilterUpdater.h"
#include "IFilterOutput.h"
#include "IFilter.h"


namespace ohmPf
{

  class FilterOutputUpdater : public FilterUpdater
  {
  public:
    FilterOutputUpdater(IFilterOutput* filterOutput, IFilter* filter);
    virtual ~FilterOutputUpdater();
    void update();
  private:
    IFilterOutput* _filterOutput;
  };

} /* namespace ohmPf */

#endif /* SRC_FILTEROUTPUTUPDATER_H_ */
