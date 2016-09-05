/*
 * ProbMap.cpp
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#include "ProbMap.h"

namespace ohmPf
{

ProbMap::ProbMap(IMap& map, unsigned int maxDistanceProbMap) :
    _map(map), _maxDistanceProbMap(maxDistanceProbMap)
{
  // dont use this function --> generate raw map from isOccupied() function.
  // we cannot rely on the data format, its up to the user.
  _mapRaw = _map.getMapData();

  calcProbMap();
}


void ProbMap::calcContourMap()
{
  int filterSize = 1;  // in cells

  _contourMap = _mapRaw;
  bool edge = false;

  for(int i = 0; i < getWidthInCells(); i++)  // x map
  {
    for(int j = 0; j < getHeighInCells(); j++)  //y map
    {
      if(_mapRaw[j * getWidthInCells() + i] == -1)
        continue;
      edge = false;
      for(int k = -filterSize; k <= filterSize; k++)  // x filter
      {
        for(int l = -filterSize; l <= filterSize; l++)  //y filter
        {
          if((i + k) >= 0 && (i + k) < getWidthInCells() && (j + l) >= 0 && (j + l) < getHeighInCells())
          {
            edge = edge
                | ((_mapRaw[j * getWidthInCells() + i] > IS_OCCUPIED_THRESHHOLD)
                    ^ (_mapRaw[(j + l) * getWidthInCells() + (i + k)] > IS_OCCUPIED_THRESHHOLD));
          }
        }
      }
      _contourMap[j * getWidthInCells() + i] = 100 * (int8_t)edge;
    }
  }

  std::cout << __PRETTY_FUNCTION__ << " --> created contour map!" << std::endl;
}

void ProbMap::calcProbMap()
{
  calcContourMap();

  int filterSize = std::ceil(1.0 * (double)_maxDistanceProbMap);  // in cells

  _probMap = _contourMap;
  double dist = 0.0;

  for(int i = 0; i < getWidthInCells(); i++)  // x map
  {
    for(int j = 0; j < getHeighInCells(); j++)  //y map
    {
      if(_contourMap[j * getWidthInCells() + i] > IS_OCCUPIED_THRESHHOLD)  // is occupied?
      {
        for(int k = -filterSize; k <= filterSize; k++)  // x filter
        {
          for(int l = -filterSize; l <= filterSize; l++)  //y filter
          {
            if((i + k) >= 0 && (i + k) < getWidthInCells() && (j + l) >= 0 && (j + l) < getHeighInCells())
            {
              dist = std::sqrt(std::pow(k, 2) + std::pow(l, 2));  // precalculate that in future
              if(dist < _maxDistanceProbMap)
              {
                dist = (1.0 - dist / _maxDistanceProbMap) * 100.0;  // normalize to [0;100]
                _probMap[(j + l) * getWidthInCells() + (i + k)] = std::max((double)_probMap[(j + l) * getWidthInCells() + (i + k)], dist);
              }
            }
          }
        }
      }
      else if(_probMap[j * getWidthInCells() + i] == -1)
      {
        _probMap[j * getWidthInCells() + i] = 0;
      }
    }
  }
  std::cout << __PRETTY_FUNCTION__ << " --> created prob map!" << std::endl;
}

double ProbMap::getProbability(Eigen::Matrix3Xd& coords, double pRand)
{
  PointInMapToOrigin(coords);

  double prob;
  double probOfCoords = 1.0;
  int x;
  int y;

  for(unsigned int i = 0; i < coords.cols(); i++)  // whole scan
  {
    if(coords(2, i) != 0)
    {
      x = meterToCells(coords(0, i));
      y = meterToCells(coords(1, i));

      if(!isInMapRange(x, y))
      {
        prob = 0.0;
      }
      else
      {
        prob = (double)_probMap[y * getWidthInCells() + x] / 100.0;  // todo: coords wird hier verändert -> das darf nicht sein!!
      }
      prob = (1 - pRand) * prob + pRand;
      probOfCoords *= prob;
    }
  }
  return probOfCoords;
}


unsigned int ProbMap::isOccupied(int x, int y)
{
  return _map.isOccupied(x, y);
}

unsigned int ProbMap::getHeighInCells()
{
  return _map.getHeighInCells();
}

unsigned int ProbMap::getWidthInCells()
{
  return _map.getWidthInCells();
}

double ProbMap::getResolution()
{
  return _map.getResolution();
}

Eigen::Matrix3d ProbMap::getTfMapToMapOrigin()
{
  return _map.getTfMapToMapOrigin();
}

std::vector<int8_t> ProbMap::getMapData()
{
  return _probMap;
}

} /* namespace ohmPf */
