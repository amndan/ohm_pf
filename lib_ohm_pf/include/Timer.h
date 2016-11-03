/*
 * Timer.h
 *
 *  Created on: Sep 16, 2016
 *      Author: amndan
 */

#ifndef LIB_OHM_PF_SRC_TIMER_H_
#define LIB_OHM_PF_SRC_TIMER_H_

#include <string>
#include <iostream>
#include <fstream>

#include <evocortex/core/time/Time.h>

namespace ohmPf
{

class Timer
{
public:
  Timer();
  Timer(std::string path);

  virtual ~Timer();

  void restart();
  void stop();
  void stopAndWrite();

  int64_t getTimeInNs();
  int64_t getTimeInMs();
  int64_t getTimeInUs();
  int64_t getTimeInSec();

private:
  void openStream();
  void writeToStream();
  evo::Time _stamp;
  evo::Duration _time;
  std::string _path;
  std::ofstream _stream;
};

} /* namespace ohmPf */

#endif /* LIB_OHM_PF_SRC_TIMER_H_ */
