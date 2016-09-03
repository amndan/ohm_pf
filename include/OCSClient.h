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
 * @brief Class for OCS functionality. (see @ref OCS)
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
   * @param resetIfTrue if true OCSClient will reset the
   * OCSFlag if it was true. This behavior is the standard
   * way to access the ocs flag:
   * --> ask for it - do sth if true - reset it
   */
  bool getOCSFlag(bool resetIfTrue = true);

  /**
   * @brief resets the OCSFlag to false.
   */
  void resetOCSFlag();

private:
  /**
   * @brief Function for setting OCS signal variable.
   * Just OCSObserver has access to this function.
   */
  void setOCSFlagTrue();

  bool _OCSFlag;
};

} /* namespace ohmPf */

#endif /* SRC_IOCSCLIENT_H_ */
