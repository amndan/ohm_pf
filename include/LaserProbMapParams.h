/*
 * LaserProbMapParams.h
 *
 *  Created on: Aug, 31 2016
 *      Author: amndan
 */

#ifndef INCLUDE_LASERPROBMAPPARAMS_H_
#define INCLUDE_LASERPROBMAPPARAMS_H_

namespace ohmPf
{

typedef struct
{
  unsigned int subsamplingLaser;
  double minValidRaysFactor;

} LaserProbMapParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_LASERPROBMAPPARAMS_H_ */

