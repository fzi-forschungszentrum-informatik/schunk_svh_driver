// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-


// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
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

// Driver Specific things
#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>

#include <boost/shared_ptr.hpp>

class SVHNode{

public:
  SVHNode(const ros::NodeHandle &nh);
  ~SVHNode();

  //! Dynamic reconfigure callback to update changing parameters
  void dynamic_reconfigure_callback(svh_controller::svhConfig &config, uint32_t level);

  //! Callback function for connecting to SCHUNK five finger hand
  void connectCallback(const std_msgs::Empty&);

  //! Callback function to reset/home channels of SCHUNK five finger hand
  void resetChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function to enable channels of SCHUNK five finger hand
  void enableChannelCallback(const std_msgs::Int8ConstPtr& channel);

  //! Callback function for setting channel target positions to SCHUNK five finger hand
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

//  //! The current controller settings as given in the parameters
//  std::vector<driver_svh::SVHCurrentSettings> current_settings_;
//  //! The postion controller settings as given in the paramters
//  std::vector<driver_svh::SVHPositionSettings> position_settings_;


};


#endif // S5FH_CONTROLLER_H
