/*
 * FilterOutputUpdater.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "FilterOutputUpdater.h"

namespace ohmPf
{

  FilterOutputUpdater::FilterOutputUpdater(IFilterOutput* filterOutput, Filter* filter) : FilterUpdater(filter)
  {
    _filterOutput = filterOutput;
  }

  FilterOutputUpdater::~FilterOutputUpdater()
  {
    // TODO Auto-generated destructor stub
  }

  Eigen::Vector3d FilterOutputUpdater::updateTf()
  {
    if(!_filter->getSampleSet()->isNormalized())
    {
      _filter->getSampleSet()->normalize();
    } // TODO: implement better routine for that

    Eigen::Vector3d pose;
    pose.setZero();

    std::vector<Sample_t>* samples = _filter->getSamples();

    for(unsigned int i = 0; i < samples->size(); i++)
    {
      pose += samples->at(i).pose * samples->at(i).weight;
    }

    pose(2) = getMeanOfAngles(*samples);

    //pose *= 1.0 / (double) samples->size();
    return pose;
  }

  void FilterOutputUpdater::update()
  {
    _filterOutput->actualizeTF(updateTf());
    _filterOutput->printSampleSet(*(_filter->getSamples()));
  }

} /* namespace ohmPf */
