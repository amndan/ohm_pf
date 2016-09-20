/*
 * MapUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "MapUpdater.h"

namespace ohmPf
{

  MapUpdater::MapUpdater(Filter* filter, IMap* map, std::string idString) :
      FilterUpdater(filter, idString)
  {
    _map = map;
  }

  void MapUpdater::update()  // update weights
  {
    std::vector<Sample_t>* samples = _filter->getSamples();

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      if(_map->isOccupied(it->pose(0), it->pose(1)) == OCC_STATES::OCCUPIED)
      {
        it->weight = 0.0;
      }
    }
  }

  void MapUpdater::initFilter()  // update weights
  {
    // init vars
    double xMin;
    double yMin;
    double xMax;
    double yMax;

    // calculate bounds
    _map->getMinEnclRect(xMin, yMin, xMax, yMax);

    // generate cloud
    std::vector<Sample_t> samples;

    for(unsigned int i = 0; i < _filter->getSamplesMax(); i++)
    {
      Sample_t sample;
      sample.weight = 1.0;
      sample.pose(2) = drand48() * 2 * M_PI - M_PI;

      // create random sample while it falls on a not occupied region
      do
      {
        sample.pose(0) = drand48() * (xMax - xMin) + xMin;
        sample.pose(1) = drand48() * (yMax - yMin) + yMin;
      }
      while(_map->isOccupied(sample.pose(0), sample.pose(1)) != OCC_STATES::FREE);
      //todo: check if there is at least one field not occupied

      samples.push_back(sample);
    }
    _filter->setSamples(samples);
  }

} /* namespace ohmPf */

