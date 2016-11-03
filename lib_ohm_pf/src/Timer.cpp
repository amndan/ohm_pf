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
  _path = "/tmp/timer";

  openStream();
  _stream.close();

  _stamp = evo::Time::now();
}

Timer::Timer(std::string path)
{
  _path = path;

  openStream();
  _stream.close();

  _stamp = evo::Time::now();
}

Timer::~Timer()
{
  _stream.close();
}

void Timer::restart()
{
  _stamp = evo::Time::now();
}

void Timer::stop()
{
  _time = evo::Time::now() - _stamp;
}

void Timer::stopAndWrite()
{
  stop();
  writeToStream();
}

void Timer::openStream()
{
  _stream.open(_path.c_str(), std::ios_base::app);

  if(!_stream.is_open())
  {
    std::runtime_error e("cannot open stream!");
    throw e;
  }
}

void Timer::writeToStream()
{
  openStream();
  _stream << (int64_t) getTimeInNs() << std::endl;
  _stream.close();
}

} /* namespace ohmPf */

int64_t ohmPf::Timer::getTimeInNs()
{
  return static_cast<int64_t>(_time.toNSec());
}

int64_t ohmPf::Timer::getTimeInUs()
{
  return static_cast<int64_t>(getTimeInNs()) / 1000ll;
}

int64_t ohmPf::Timer::getTimeInMs()
{
  return static_cast<int64_t>(getTimeInUs()) / 1000ll;
}

int64_t ohmPf::Timer::getTimeInSec()
{
  return static_cast<int64_t>(getTimeInMs()) / 1000ll;
}


