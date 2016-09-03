/*
 * OCSClient.h
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
class OCSClient
{
  friend class OCSObserver;

public:
  /**
   * @brief Constructor
   */
  OCSClient();

  /**
   * @brief Destructor
   */
  virtual ~OCSClient(){};

protected:
  /**
   * @brief Return OCSFlags state to inheritand objects.
   * @return true if odom changed significantly.
   */
  bool getOCSFlag(bool resetIfTrue = true);

private:
  /**
   * @brief Function for setting OCS signal variable.
   */
  void setOCSFlagTrue();

  bool _OCSFlag;
};

} /* namespace ohmPf */

#endif /* SRC_IOCSCLIENT_H_ */
