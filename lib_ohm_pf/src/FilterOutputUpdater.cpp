/*
 * FilterOutputUpdater.cpp
 *
 *  Created on: 09.03.2016
 *      Author: amndan
 */

#include "FilterOutputUpdater.h"

namespace ohmPf
{

  FilterOutputUpdater::FilterOutputUpdater(IFilterOutput* filterOutput, Filter* filter, std::string idString) :
      FilterUpdaterTimed( filter, ros::Duration(filter->getParams().outputIntervall), idString)
  {
    _filterOutput = filterOutput;
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
    _filterOutput->onOutputPoseChanged(updateTf(), _filter->getStamp());
    _filterOutput->onSampleSetChanged(*(_filter->getSamples()));

    // update prob of samples TODO: this one should go elsewhere
    // filter state is more than prob; its sample count etc. updating this values should not happen here
    _filter->getFilterState()->probPose = getQualityOfSamples(*(_filter->getSamples()));

    _filterOutput->onFilterStateChanged(*(_filter->getFilterState()));
  }

} /* namespace ohmPf */