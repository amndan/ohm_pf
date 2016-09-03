/*
 * OCSClient.cpp
 *
 *  Created on: Sep 3, 2016
 *      Author: amndan
 */

#include <OCSClient.h>

namespace ohmPf
{

  OCSClient::OCSClient()
  {
    _OCSFlag = false;
  }

  void OCSClient::setOCSFlagTrue()
  {
    _OCSFlag = true;
  }

  bool OCSClient::getOCSFlag(bool resetIfTrue)
  {
    if(resetIfTrue && _OCSFlag)
    {
      _OCSFlag = false;
      return true;
    }
    else
    {
      return false;
    }
  }
}
