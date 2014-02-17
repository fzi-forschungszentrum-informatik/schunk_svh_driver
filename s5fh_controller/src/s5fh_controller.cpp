// ROS includes.
#include <ros/ros.h>

// Messages
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <s5fh_controller/s5fhConfig.h>

#include <driver_s5fh/S5FHFingerManager.h>

using namespace driver_s5fh;

// Create pointer to S5FH finger manager object.
S5FHFingerManager *fm = new S5FHFingerManager;

// Set default serial device name
std::string serial_device_name = "";

/*--------------------------------------------------------------------
 * General functions
 *------------------------------------------------------------------*/

void setTargetPositions(const std::vector<double>& positions)
{
  if (fm->isConnected())
  {
    for (size_t channel = 0; channel < eS5FH_DIMENSION; ++channel)
    {
      double cur_pos = 0.0;
      if (fm->getPosition(static_cast<S5FHCHANNEL>(channel), cur_pos))
      {
        if (positions[channel] != cur_pos)
        {
          //ROS_INFO("channel %i: current pos = %f, new pos = %f", channel, cur_pos, positions[channel]);
          fm->setTargetPosition(static_cast<S5FHCHANNEL>(channel), positions[channel], 0.0);
        }
      }
      else
      {
        ROS_WARN("Could not get actual position from finger manager!");
      }
    }
  }
  else
  {
    ROS_WARN("SCHUNK five finger hand is not connected!");
  }
}

/*--------------------------------------------------------------------
 * Callback functions
 *------------------------------------------------------------------*/

// Callback function for changing parameters dynamically
void dynamic_reconfigure_callback(s5fh_controller::s5fhConfig &config, uint32_t level)
{
  serial_device_name = config.serial_device;

  std::vector<double> positions(eS5FH_DIMENSION);
  positions[0] = config.channel0;
  positions[1] = config.channel1;
  positions[2] = config.channel2;
  positions[3] = config.channel3;
  positions[4] = config.channel4;
  positions[5] = config.channel5;
  positions[6] = config.channel6;
  positions[7] = config.channel7;
  positions[8] = config.channel8;

  setTargetPositions(positions);
}

// Callback function for connecting to SCHUNK five finger hand
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

// Callback function to reset/home channels of SCHUNK five finger hand
void resetChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  // convert int8 channel into S5FHCHANNEL enum
  S5FHCHANNEL s5fh_channel = static_cast<S5FHCHANNEL>(channel->data);

  if (fm->resetChannel(s5fh_channel))
  {
    ROS_INFO("Channel %i successfully homed!", s5fh_channel);
  }
  else
  {
    ROS_ERROR("Could not reset channel %i !", s5fh_channel);
  }
}

// Callback function to enable channels of SCHUNK five finger hand
void enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
//  fm->enableChannel(static_cast<driver_s5fh::S5FHCHANNEL>(channel->data));
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "s5fh_controller");
  ros::NodeHandle nh;

  icl_core::logging::initialize();

  // setting up dynamic reconfigure
  dynamic_reconfigure::Server<s5fh_controller::s5fhConfig> server;
  dynamic_reconfigure::Server<s5fh_controller::s5fhConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, connectCallback);

  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub = nh.subscribe("reset_channel", 1, resetChannelCallback);

  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub = nh.subscribe("enable_channel", 1, enableChannelCallback);

  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(100);

  // Main loop.
  while (nh.ok())
  {
    if (fm->isConnected())
    {
//      // TODO: Publish joint angle topic.
//      for (size_t channel = 0; channel < eS5FH_DIMENSION; ++channel)
//      {
//        double cur_pos = 0.0;
//        if (fm->getPosition(static_cast<S5FHCHANNEL>(channel), cur_pos))
//        {
//          std::cout << "channel: " << (int)channel << " pos = " << cur_pos << " | ";
//        }
//      }
//      std::cout << std::endl;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
} // end main()
