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
      resetOCSFlag();
      return true;
    }
    else
    {
      return false;
    }
  }

  void OCSClient::resetOCSFlag()
  {
    _OCSFlag = false;
  }

}
