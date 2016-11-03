/*
 * IMap.h
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#ifndef SRC_IMAP_H_
#define SRC_IMAP_H_

#include "Eigen/Dense"

namespace ohmPf
{
namespace OCC_STATES
{
  /**
   * @brief Enum for occupancy states. Free and Occupied was not enough.
   */
  enum
  {
    FREE = 0,    //!< FREE
    OCCUPIED = 1,//!< OCCUPIED
    UNKNOWN = 2, //!< UNKNOWN
    OUTBOUND = 3 //!< OUTBOUND
  };
}

/**
 * @brief An interface class for Maps. The user must provide a map implementing
 * this interface for the filter.
 */
class IMap
{

public:
  IMap(){};
  virtual ~IMap(){};

  /**
   * @brief Get occupancy state from a given cell.
   * @param x cell nr in x direction
   * @param y cell nr in y direction
   * @return returns one of OCC_STATES enum
   */
  virtual unsigned int isOccupied(int x, int y) = 0;  //x, y in cells

  /**
   * @return height of map in cells
   */
  virtual unsigned int getHeighInCells() = 0;

  /**
   * @return width of map in cells
   */
  virtual unsigned int getWidthInCells() = 0;

  /**
   * @return resolution of map in m/cells
   */
  virtual double getResolution() = 0; //res in m/cell

  /**
   * @brief Origin of the map is not fix, but origin of the occupancy array is (cell 0 | cell 0)
   * This function returns a transformation matrix from the maps origin to the arrays origin.
   * Its unit is meter and rad.
   * @return The transformation from the maps origin to the arrays origin in meter.
   */
  virtual Eigen::Matrix3d getTfMapToMapOrigin() = 0;

  /**
   * @return The maps data vector.
   */
  virtual std::vector<int8_t> getMapData() = 0;

  /**
   * @brief Get occupancy state from a given cell.
   * @param x value in m
   * @param y value in m
   * @return returns one of OCC_STATES enum
   */
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

  /**
   * @return returns the maps height in meter.
   */
  double getHeighInMeter();

  /**
   * @return returns the maps width in meter.
   */
  double getWidthInMeter();

  /**
   * @brief Check if the cells are inside map arrays range
   * @param x in cells
   * @param y in cells
   * @return true if is in map range.
   */
  bool isInMapRange(int x, int y);

  /**
   * @brief Converts coordinates in map coordinate system to cells.
   * @param x position in m
   * @param y position in m
   */
  void PointInMapToOrigin(double& x, double& y);

  /**
   * @brief Converts a coordinates vector in map coordinate system to cells.
   * @param coords Vector of chords - format:
   * x: x value
   * y: y value
   * 1: homogeneous coordinates
   */
  void PointInMapToOrigin(Eigen::Matrix3Xd& coords); // in meter

  /**
   * @brief convert a meter value to cells
   * @param x in meter
   * @return corresponding cells according to resolution.
   */
  int meterToCells(double x);
};

} /* namespace ohmPf */

#endif /* SRC_IMAP_H_ */
