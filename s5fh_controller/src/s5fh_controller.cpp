// ROS includes.
#include "ros/ros.h"

#include <driver_s5fh/S5FHFingerManager.h>

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

using driver_s5fh::S5FHFingerManager;

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "s5fh_controller");
  ros::NodeHandle n;

  icl_core::logging::initialize();

  // set default serial device name
  std::string serial_device_name = "/dev/ttyUSB0";

  // Create a new S5FH FingerManager object.
  S5FHFingerManager *finger_manager = new S5FHFingerManager(serial_device_name);

  // TODO: Subscribe topics

  // Tell ROS how fast to run this node. (100 Hz = 10 ms)
  ros::Rate r(100);

  // Main loop.
  while (n.ok())
  {
    // TODO: Publish topic.

    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
