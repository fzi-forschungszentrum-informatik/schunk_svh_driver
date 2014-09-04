// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-


// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2014-8-22
 *
 */
//----------------------------------------------------------------------

#ifndef S5FH_NODE_H
#define S5FH_NODE_H

// Messages
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <svh_controller/svhConfig.h>
#include <svh_controller/svhFingerConfig.h>

#include <driver_svh/SVHFingerManager.h>


#include <boost/shared_ptr.hpp>

class SVHNode{

public:
  SVHNode(const bool &autostart = false,const std::vector<bool> & disable_flags_vec = std::vector<bool>(false,driver_svh::eSVH_DIMENSION));
  ~SVHNode();

  void dynamic_reconfigure_callback(svh_controller::svhConfig &config, uint32_t level);

  void dynamic_reconfigure_callback_finger(svh_controller::svhFingerConfig &config, uint32_t level, const uint32_t &finger);

  // Callback function for connecting to SCHUNK five finger hand
  void connectCallback(const std_msgs::Empty&);

  // Callback function to reset/home channels of SCHUNK five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  // Callback function to enable channels of SCHUNK five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  // Callback function for setting channel target positions to SCHUNK five finger hand
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& input);

  //!
  //! \brief SVHNode::getCurrentPositions Gets the latest received positions from the driver
  //! \returns The current positions of all channels in rad
  //!
  sensor_msgs::JointState getCurrentPositions();


private:
  //! Handle to the SVH finger manager for hardware acces
  boost::shared_ptr<driver_svh::SVHFingerManager> fm_;

  //! Serial device to use for communication with hardware
  std::string serial_device_name_;

  //! joint state message template
  sensor_msgs::JointState channel_pos_;


};


#endif // S5FH_CONTROLLER_H
