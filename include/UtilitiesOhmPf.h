/*
 * UtilitiesOhmPf.h
 *
 *  Created on: Jan 9, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_UTILITIESOHMPF_H_
#define INCLUDE_UTILITIESOHMPF_H_

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

#endif /* INCLUDE_UTILITIESOHMPF_H_ */
