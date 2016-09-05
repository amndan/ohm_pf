/*
 * IMap.h
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#ifndef SRC_IMAP_H_
#define SRC_IMAP_H_

#include "Eigen/Dense"
#include <ros/time.h>

namespace ohmPf
{
namespace OCC_STATES
{
  enum
  {
    FREE = 0,
    OCCUPIED = 1,
    UNKNOWN = 2,
    OUTBOUND = 3
  };
}

class IMap
{

public:
  IMap(){};
  virtual ~IMap(){};
  virtual unsigned int isOccupied(int x, int y) = 0;  //x, y in cells
  virtual unsigned int getHeighInCells() = 0;
  virtual unsigned int getWidthInCells() = 0;
  virtual double getResolution() = 0; //res in m/cell
  virtual Eigen::Matrix3d getTfMapToMapOrigin() = 0;
  virtual std::vector<int8_t> getMapData() = 0;

  unsigned int isOccupied(double x, double y);  //x, y in m
  /**
   * @brief Calculates coordinates for generating random samples all over the map.
   * Map origin in map frame is variable, so before sampling for the whole map
   * x and y bounds must be known.
   * @param xMin minimum x bound in m.
   * @param yMin minimum y bound in m.
   * @param xMax maximum x bound in m.
   * @param yMax maximum y bound in m.
   */
  void getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax);  // in meter
  double getHeighInMeter();
  double getWidthInMeter();
  bool isInMapRange(int x, int y);
  void PointInMapToOrigin(double& x, double& y);
  void PointInMapToOrigin(Eigen::Matrix3Xd& coords); // in meter
  int meterToCells(double x);
};

} /* namespace ohmPf */

#endif /* SRC_IMAP_H_ */
