/*
 * FilterOutputUpdater.h
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#ifndef SRC_FILTEROUTPUTUPDATER_H_
#define SRC_FILTEROUTPUTUPDATER_H_

#include "FilterUpdater.h"
#include "Filter.h"
#include "IFilterOutput.h"
#include "Eigen/Dense"
#include "assert.h"
#include "UtilitiesOhmPf.h"


namespace ohmPf
{

  class FilterOutputUpdater : public FilterUpdater
  {
  public:
    FilterOutputUpdater(IFilterOutput* filterOutput, Filter* filter);
    virtual ~FilterOutputUpdater();
    void update();
  private:
    IFilterOutput* _filterOutput;
    Eigen::Vector3d updateTf();
  };

} /* namespace ohmPf */

#endif /* SRC_FILTEROUTPUTUPDATER_H_ */
