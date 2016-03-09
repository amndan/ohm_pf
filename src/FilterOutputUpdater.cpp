/*
 * FilterOutputUpdater.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "FilterOutputUpdater.h"

namespace ohmPf
{

  FilterOutputUpdater::FilterOutputUpdater(IFilterOutput& filterOutput)
  {
    _filterOutput = filterOutput;
  }

  FilterOutputUpdater::~FilterOutputUpdater()
  {
    // TODO Auto-generated destructor stub
  }

  void FilterOutputUpdater::update()
  {
    _filterOutput->actualizeTF();
    _filterOutput->printSampleSet();
  }

} /* namespace ohmPf */
