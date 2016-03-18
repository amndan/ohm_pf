/*
 * OCSObserver.h
 *
 *  Created on: Mar 18, 2016
 *      Author: amndan
 */

#ifndef SRC_OCSOBSERVER_H_
#define SRC_OCSOBSERVER_H_

#include "interfaces/IOCSClient.h"
#include <vector>
#include "assert.h"

namespace ohmPf
{

class OCSObserver
{
public:
  OCSObserver();
  virtual ~OCSObserver();
  void registerClient(IOCSClient* client, double dist);
private:
  std::vector<IOCSClient*> _clientList;
  std::vector<double> _dists;
};

} /* namespace ohmPf */

#endif /* SRC_OCSOBSERVER_H_ */
