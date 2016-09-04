/*
 * ProbMap.h
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#ifndef SRC_PROBMAP_H_
#define SRC_PROBMAP_H_

#include <IMap.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#define IS_OCCUPIED_THRESHHOLD 50
#define IS_UNKNOWN_CELL -1

namespace ohmPf
{

class ProbMap : public IMap
{
public:
  ProbMap(IMap& map, unsigned int maxDistanceProbMap);
  virtual ~ProbMap();

  unsigned int isOccupied(int x, int y);  //x, y in cells
  unsigned int getHeighInCells();
  unsigned int getWidthInCells();
  double getResolution(); //res in m/cell
  Eigen::Matrix3d getTfMapToMapOrigin();
  std::vector<int8_t> getMapRaw();

  double getProbability(Eigen::Matrix3Xd& coords, double pRand);
  double getProbability(double x, double y);

private:
  void calcContourMap();
  void calcProbMap();

  IMap& _map;

  unsigned int _maxDistanceProbMap;

  std::vector<int8_t> _mapRaw;
  std::vector<int8_t> _probMap;
  std::vector<int8_t> _contourMap;
};

} /* namespace ohmPf */

#endif /* SRC_PROBMAP_H_ */
