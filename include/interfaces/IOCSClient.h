/*
 * IOCSClient.h
 *
 *  Created on: Mar 10, 2016
 *      Author: amndan
 */

#ifndef SRC_IOCSCLIENT_H_
#define SRC_IOCSCLIENT_H_

namespace ohmPf
{

class IOCSClient
{
public:
  IOCSClient(){};
  virtual ~IOCSClient(){};
  virtual void setOCSFlagTrue() = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IOCSCLIENT_H_ */
