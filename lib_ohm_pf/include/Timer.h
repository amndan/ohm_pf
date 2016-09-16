/*
 * Timer.h
 *
 *  Created on: Sep 16, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_TIMER_H_
#define LIB_OHM_PF_SRC_TIMER_H_

#include <string>
#include <ros/time.h>
#include <iostream>
#include <fstream>

namespace ohmPf
{

class Timer
{
public:
  Timer();
  Timer(std::string path);

  virtual ~Timer();

  void start();
  void stop();

private:
  void writeToStream();
  ros::Time _stamp;
  std::string _path;
  std::ofstream _stream;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_TIMER_H_ */
