/*
 * LaserProbMapMethod.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "LaserProbMapMethod.h"

namespace ohmPf
{

LaserProbMapMethod::LaserProbMapMethod( double minValidRaysFactor )
{

  if(minValidRaysFactor > 1.0 || minValidRaysFactor <= 0.0)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> minValidRaysFactor must be in between intervall ]0.0;1.0]"
        " Requested Factor was: " << minValidRaysFactor << std::endl;

    std::cout << __PRETTY_FUNCTION__ << "--> Will proceed with Factor of 0.5" << std::endl;
        _minValidRaysFactor = 0.5;
  }
  else
  {
    _minValidRaysFactor = minValidRaysFactor;
  }

}

Eigen::Matrix3Xd LaserProbMapMethod::rangesToCoordinates(ILaserMeasurement& measurement)
{
  std::vector<float> ranges = measurement.getRanges();


  // check ranges.size
  if (ranges.size() != measurement.getCount())
  {
    std::cout << __PRETTY_FUNCTION__ << "--> ranges.size() != measurement.getCount()!" << std::endl;
    std::cout << "Ranges.size = " << ranges.size() << " Count = " << measurement.getCount() << std::endl;
  }

  // calculate steps
  int iter = (int)std::floor(ranges.size() / measurement.getSubsamplingRate());

  // init vars
  Eigen::Matrix3Xd scanCoord(3, iter + 1); // why iter+1: (int) abs(5 / 2) = 2 --> 0 2 4 --> 2 + 1 = 3
  unsigned int invalidRayCounter = 0;

  // transform ranges to coordinates in laser frame
  for (unsigned int i = 0, j = 0; i < ranges.size(); i = i + measurement.getSubsamplingRate(), j++)
  {
    // filter measurements
    if (ranges[i] <= measurement.getRangeMax() && ranges[i] >= measurement.getRangeMin() && !std::isinf(ranges[i]))
    {
      scanCoord(0, j) = ranges[i] * std::cos(measurement.getAngleMin() + i * measurement.getAngleIncrement());
      scanCoord(1, j) = ranges[i] * std::sin(measurement.getAngleMin() + i * measurement.getAngleIncrement());
      scanCoord(2, j) = 1;
    }
    // here comes inf and out of range measurements
    else
    {
      scanCoord(0, j) = 0.0;
      scanCoord(1, j) = 0.0;
      scanCoord(2, j) = 0.0;  // used as mark for invalid scans
      invalidRayCounter++;  // increment
    }
  }

  // calculate invalid scans factor and send debug message
  if(1.0 - ( invalidRayCounter / (iter + 1) ) < _minValidRaysFactor)
  {
    std::cout << __PRETTY_FUNCTION__ << "--> Too less rays for updating filter. This is just a debugging message,"
        "The feature itsel is not implemented yet. invalidRayCounter = " << invalidRayCounter << std::endl;
  }

  return scanCoord;
}

void LaserProbMapMethod::calculate(Filter& filter, ILaserMeasurement& measurement, IMap& map,
                                   MapUpdater* updateFilterMap)
{
  Eigen::Matrix3Xd coords = rangesToCoordinates(measurement);

  std::vector<Sample_t>* samples = filter.getSamples();

  Eigen::Matrix3Xd coordsTf;
  Eigen::Matrix3d tf;

  for (std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it) // each sample
  {
    // transform scan to position of particle
    create3x3TransformationMatrix(it->pose(0), it->pose(1), it->pose(2), tf); // todo: dont forget laser tf
    coordsTf = tf * measurement.getTfBaseFootprintToLaser() * coords;

    // lookup probs
    it->weight = map.getProbability(coordsTf, measurement.getUncertainty());
  }

  filter.getSampleSet()->boostWeights();
  if (updateFilterMap != NULL)
    updateFilterMap->update();
  filter.getSampleSet()->normalize();

  //filter.getSampleSet()->normalize();
  //filter.getSampleSet()->resample(); // todo: should we do that here??
}

} /* namespace ohmPf */
