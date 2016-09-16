/*
 * Timer.cpp
 *
 *  Created on: Sep 16, 2016
 *      Author: amndan
 */

#include "Timer.h"

namespace ohmPf
{

Timer::Timer()
{
  Timer("/tmp/timer");
}

Timer::Timer(std::string path)
{
  _path = path;

  _stream.open(_path.c_str());

  if(!_stream.is_open())
  {
    throw std::runtime_error;
  }

  _stream.close();
}

Timer::~Timer()
{
  // TODO Auto-generated destructor stub
}

void Timer::start()
{
}

void Timer::stop()
{
}

void Timer::writeToStream()
{
  // open write close
}

} /* namespace ohmPf */
