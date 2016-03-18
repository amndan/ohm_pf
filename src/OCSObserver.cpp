/*
 * OCSObserver.cpp
 *
 *  Created on: Mar 18, 2016
 *      Author: amndan
 */

#include "OCSObserver.h"

namespace ohmPf
{

OCSObserver::OCSObserver()
{
  // TODO Auto-generated constructor stub

}

OCSObserver::~OCSObserver()
{
  // TODO Auto-generated destructor stub
}

void registerClient(IOCSClient* client, double dist)
{
  assert(client != NULL);
  assert(dist >= 0); 

  _clientList.pushBack(client);
  _dists.pushBack.(dist);
}

} /* namespace ohmPf */
