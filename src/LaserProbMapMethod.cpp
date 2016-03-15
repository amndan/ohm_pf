/*
 * LaserProbMapMethod.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "LaserProbMapMethod.h"

namespace ohmPf
{

  LaserProbMapMethod::LaserProbMapMethod()
  {
    // TODO Auto-generated constructor stub

  }

  LaserProbMapMethod::~LaserProbMapMethod()
  {
    // TODO Auto-generated destructor stub
  }

  Eigen::Matrix3Xd LaserProbMapMethod::rangesToCoordinates(ILaserMeasurement& measurement)
  // TODO: this function implements a kind of a laser filter; this function should be elsewhere
  {
    std::vector<float> ranges = measurement.getRanges();

    if(ranges.size() != measurement.getCount())
    {
      std::cout << ranges.size() << "|" << measurement.getCount() << std::endl;
    }
      assert(ranges.size() == measurement.getCount());


    int subSampFact = 3;  // todo: use parameter --> subsampling in laserFilter?

    int iter = (int)std::floor(measurement.getCount() / subSampFact);

    Eigen::Matrix3Xd scanCoord(3, iter + 1);  // abs(5 / 2) = 2 --> 0 2 4 --> 2 + 1 = 3

    for(unsigned int i = 0, j = 0; i < measurement.getCount(); i = i + subSampFact, j++)
    {
      if(ranges[i] <= measurement.getRangeMax() && ranges[i] >= measurement.getRangeMin() && !std::isinf(ranges[i]))  //todo: laserfilter
      {
        scanCoord(0, j) = ranges[i] * std::cos(measurement.getAngleMin() + i * measurement.getAngleIncrement());
        scanCoord(1, j) = ranges[i] * std::sin(measurement.getAngleMin() + i * measurement.getAngleIncrement());
        scanCoord(2, j) = 1;
      }
      else
      {
        scanCoord(0, j) = 0.0;  // todo: perhaps use a mask here
        scanCoord(1, j) = 0.0;
        scanCoord(2, j) = 0.0;
      }
    }

    return scanCoord;
  }

  void LaserProbMapMethod::calculate(Filter& filter, ILaserMeasurement& measurement, IMap& map, MapUpdater* updateFilterMap)
  {
    Eigen::Matrix3Xd coords = rangesToCoordinates(measurement);

    std::vector<Sample_t>* samples = filter.getSamples();

    Eigen::Matrix3Xd coordsTf;
    Eigen::Matrix3d tf;

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)  // each sample
    {
      // transform scan to position of particle
      create3x3TransformationMatrix(it->pose(0), it->pose(1), it->pose(2), tf);  // todo: dont forget laser tf
      coordsTf = tf * measurement.getTfBaseFootprintToLaser() * coords;

      // lookup probs
      it->weight = map.getProbability(coordsTf, measurement.getUncertainty());
    }

    filter.getSampleSet()->boostWeights();
    if(updateFilterMap != NULL) updateFilterMap->update();
    filter.getSampleSet()->normalize();

    //filter.getSampleSet()->normalize();
    //filter.getSampleSet()->resample(); // todo: should we do that here??
  }

} /* namespace ohmPf */
