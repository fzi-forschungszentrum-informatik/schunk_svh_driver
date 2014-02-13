// ROS includes.
#include "ros/ros.h"

#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"

#include <driver_s5fh/S5FHFingerManager.h>

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

using driver_s5fh::S5FHFingerManager;

// Create pointer to S5FH finger manager object.
S5FHFingerManager *fm = new S5FHFingerManager;

// Set default serial device name
std::string serial_device_name = "/dev/ttyUSB1";

void connectCallback(const std_msgs::Empty&)
{
  if (fm->isConnected())
  {
    fm->disconnect();
  }

  if (!fm->connect(serial_device_name))
  {
    ROS_ERROR("Could not connect to SCHUNK five finger hand with serial device %s", serial_device_name.c_str());
  }
}

void resetChannelCallback(const std_msgs::UInt8ConstPtr& channel)
{
  fm->resetChannel(static_cast<driver_s5fh::S5FHCHANNEL>(channel->data));
}

void enableChannelCallback(const std_msgs::UInt8ConstPtr& channel)
{
//  fm->enableChannel(static_cast<driver_s5fh::S5FHCHANNEL>(channel->data));
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "s5fh_controller");
  ros::NodeHandle nh;

  icl_core::logging::initialize();

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, connectCallback);

  // Subscribe reset channel topic (UInt8)
  ros::Subscriber reset_sub = nh.subscribe("reset_channel", 1, resetChannelCallback);

  // Subscribe enable channel topic (UInt8)
  ros::Subscriber enable_sub = nh.subscribe("enable_channel", 1, enableChannelCallback);

  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(100);

  // Main loop.
  while (nh.ok())
  {
    // TODO: Publish joint angle topic.

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
} // end main()
