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

/**
 * @brief Interface for using OCS functionality.
 */
class IOCSClient
{
public:
  /**
   * @brief Constructor
   */
  IOCSClient(){};

  /**
   * @brief Destructor
   */
  virtual ~IOCSClient(){};

  /**
   * @brief Abstract function for OCS signal
   */
  virtual void setOCSFlagTrue() = 0;
};

} /* namespace ohmPf */

#endif /* SRC_IOCSCLIENT_H_ */
