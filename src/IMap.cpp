/*
 * IMap.cpp
 *
 *  Created on: Sep 4, 2016
 *      Author: amndan
 */

#include "IMap.h"

namespace ohmPf
{

void IMap::getMinEnclRect(double& xMin, double& yMin, double& xMax, double& yMax)
{
  Eigen::MatrixXd rectInOrigin(3, 4);
  rectInOrigin.col(0) << 0, 0, 1;
  rectInOrigin.col(1) << getWidthInMeter(), 0, 1;
  rectInOrigin.col(2) << 0, getHeighInMeter(), 1;
  rectInOrigin.col(3) << getWidthInMeter(), getHeighInMeter(), 1;

  rectInOrigin = getTfMapToMapOrigin() * rectInOrigin;

  xMax = rectInOrigin.row(0).maxCoeff();
  xMin = rectInOrigin.row(0).minCoeff();
  yMax = rectInOrigin.row(1).maxCoeff();
  yMin = rectInOrigin.row(1).minCoeff();
}

bool IMap::isInMapRange(int x, int y)
{
  if(x < 0 || y < 0 || x >= (int) getWidthInCells() || y >= (int) getHeighInCells())
  {
    return false;
  }
  else
  {
    return true;
  }
}

unsigned int IMap::isOccupied(double x, double y)  //x, y in m
{
  PointInMapToOrigin(x, y);

  int x_c = meterToCells(x);
  int y_c = meterToCells(y);

  return isOccupied(x_c, y_c);
}

void IMap::PointInMapToOrigin(double& x, double& y)
{
  Eigen::Vector3d point;
  point(0) = x;
  point(1) = y;
  point(2) = 1;

  point = getTfMapToMapOrigin().inverse() * point;

  x = point(0);
  y = point(1);

  return;
}

void ohmPf::IMap::PointInMapToOrigin(Eigen::Matrix3Xd& coords)
{
  coords = getTfMapToMapOrigin().inverse() * coords;
}

int IMap::meterToCells(double x)
{
  int ret = std::floor(x / getResolution());
  return ret;
}

double IMap::getHeighInMeter()
{
  return getHeighInCells() * getResolution();
}

double IMap::getWidthInMeter()
{
  return getWidthInCells() * getResolution();
}


} /* namespace ohmPf */


