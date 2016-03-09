/*
 * RosLaserPMParams.h
 *
 *  Created on: Jan 8, 2016
 *      Author: amndan
 */

#ifndef INCLUDE_RosLaserPMParams_t_H_
#define INCLUDE_RosLaserPMParams_t_H_

namespace ohmPf
{

typedef struct
{
  double angleIncrement;
  double rangeMax;
  double rangeMin;
  double angleMin;
  double angleMax;
  unsigned int count;
  unsigned int subsamplingRate;
  double uncertainty; // range: [0 1[
  std::string tfLaserFrame;
  std::string tfBaseFooprintFrame;
} RosLaserPMParams_t;

} /* namespace ohmPf */

#endif /* INCLUDE_OdomDiffParams_t_H_ */