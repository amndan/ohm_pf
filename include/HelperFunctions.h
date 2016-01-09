/*
 * HelperFunctions.h
 *
 *  Created on: Jan 9, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_HELPERFUNCTIONS_H_
#define INCLUDE_HELPERFUNCTIONS_H_

#include <cmath>

namespace ohmPf
{

static void correctAngleOverflow(double& angle)
{
  angle = std::fmod(angle, 2 * M_PI);

  if(angle > M_PI)
    angle -= 2 * M_PI;
  else if(angle < -M_PI)
    angle += 2 * M_PI;
}

}  // ohmPf

#endif /* INCLUDE_HELPERFUNCTIONS_H_ */
