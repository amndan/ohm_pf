/*
 * MapUpdater.cpp
 *
 *  Created on: 11.03.2016
 *      Author: amndan
 */

#include "MapUpdater.h"

namespace ohmPf
{

  MapUpdater::MapUpdater(Filter* filter, IMap* map) :
      FilterUpdater(filter)
  {
    _map = map;
  }

  void MapUpdater::update()  // update weights
  {
    std::vector<Sample_t>* samples = _filter->getSamples();

    for(std::vector<Sample_t>::iterator it = samples->begin(); it != samples->end(); ++it)
    {
      if(_map->isOccupied(it->pose(0), it->pose(1)))
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
    getMinEnclRect(xMin, yMin, xMax, yMax);

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
      while(_map->isOccupied(sample.pose(0), sample.pose(1)));
      //todo: check if there is at least one field not occupied

      samples.push_back(sample);
    }
    _filter->setSamples(samples);
  }

  void MapUpdater::getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax)
  {
    Eigen::MatrixXd rectInOrigin(3, 4);
    rectInOrigin.col(0) << 0, 0, 1;
    rectInOrigin.col(1) << _map->getWith(), 0, 1;
    rectInOrigin.col(2) << 0, _map->getHeigh(), 1;
    rectInOrigin.col(3) << _map->getWith(), _map->getHeigh(), 1;

    rectInOrigin = _map->getTfMapToMapOrigin() * rectInOrigin;

    xMax = rectInOrigin.row(0).maxCoeff();
    xMin = rectInOrigin.row(0).minCoeff();
    yMax = rectInOrigin.row(1).maxCoeff();
    yMin = rectInOrigin.row(1).minCoeff();
  }


} /* namespace ohmPf */

