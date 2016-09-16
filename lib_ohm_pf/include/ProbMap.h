/*
 * ProbMap.h
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#ifndef SRC_PROBMAP_H_
#define SRC_PROBMAP_H_

#include <IMap.h>

#define IS_OCCUPIED_THRESHHOLD 50
#define IS_UNKNOWN_CELL -1

namespace ohmPf
{

/**
 * @brief ProbMap is a extension to an IMap. It uses the Decorator pattern
 * to extend an, at compile time unknown, IMap instance with Probability Map
 * functionality.
 */
class ProbMap : public IMap
{
public:
  /**
   * @brief Constructor initializes the ProbMap with the IMap instance.
   * @param map Pointer to the IMap instance.
   * @param maxDistanceProbMap The kernel size of the prob map.
   */
  ProbMap(IMap& map, unsigned int maxDistanceProbMap);

  /**
   * Destructor (empty)
   */
  virtual ~ProbMap(){};


  /**
   * @brief Delegated to IMap instance.
   */
  unsigned int isOccupied(int x, int y);  //x, y in cells

  /**
   * @brief Delegated to IMap instance.
   */
  unsigned int getHeighInCells();

  /**
   * @brief Delegated to IMap instance.
   */
  unsigned int getWidthInCells();

  /**
   * @brief Delegated to IMap instance.
   */
  double getResolution(); //res in m/cell

  /**
   * @brief Delegated to IMap instance.
   */
  Eigen::Matrix3d getTfMapToMapOrigin();

  /**
   * @brief returns the map data from the prob map.
   * @return Data array.
   * -1:        unknown
   * 0-100:     probability mapped from 0.0-1.0 to 0-100
   */
  std::vector<int8_t> getMapData();

  /**
   * @brief Get probability values for a pose array.
   * @param coords Pose coordinates - format:
   * x y valid with units: m m a.u
   * z value is used as validity mask. 0 means this pose
   * is not valid whereas it does not get processed.
   * @param pRand The random probability to add.
   * @return summed up probability of all poses.
   */
  double getProbability(Eigen::Matrix3Xd& coords, double pRand);

private:
  /**
   * @brief Helper function to calculate a countour map from the input map.
   * It extracts the contours of the input image.
   */
  void calcContourMap();

  /**
   * @brief It calculates the probability map from the contour map.
   * @todo Use gaussian probability here.
   * @todo Dont use IS_OCCUPIED_THRESHHOLD macros here. Generate own map
   * sata and use OCC_STATES here.
   */
  void calcProbMap();

  IMap& _map;

  unsigned int _maxDistanceProbMap;

  std::vector<int8_t> _mapRaw;
  std::vector<int8_t> _probMap;
  std::vector<int8_t> _contourMap;
};

} /* namespace ohmPf */

#endif /* SRC_PROBMAP_H_ */
