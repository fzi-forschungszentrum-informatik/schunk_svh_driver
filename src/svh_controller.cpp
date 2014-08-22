// ROS includes.
#include <ros/ros.h>

// Messages
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <svh_controller/svhConfig.h>

#include <driver_svh/SVHFingerManager.h>
#include <svh_controller.h>
#include <boost/shared_ptr.hpp>

#include <icl_core/EnumHelper.h>

using namespace driver_svh;

// Create pointer to SVH finger manager object.
boost::shared_ptr<SVHFingerManager> fm;

// Set default serial device name
std::string serial_device_name = "";

/*--------------------------------------------------------------------
 * Callback functions
 *------------------------------------------------------------------*/

// Callback function for changing parameters dynamically
void dynamic_reconfigure_callback(svh_controller::svhConfig &config, uint32_t level)
{
  serial_device_name = config.serial_device;
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
  // convert int8 channel into SVHChannel enum
  SVHChannel svh_channel = static_cast<SVHChannel>(channel->data);

  if (fm->resetChannel(svh_channel))
  {
    ROS_INFO("Channel %i successfully homed!", svh_channel);
  }
  else
  {
    ROS_ERROR("Could not reset channel %i !", svh_channel);
  }
}

// Callback function to enable channels of SCHUNK five finger hand
void enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  fm->enableChannel(static_cast<driver_svh::SVHChannel>(channel->data));
}

// Callback function for setting channel target positions to SCHUNK five finger hand
void jointStateCallback(const sensor_msgs::JointStateConstPtr& input )
{
  // vector to read target positions from joint states
  std::vector<double> target_positions(eSVH_DIMENSION, 0.0);
  // bool vector storing true, if new target position read
  std::vector<bool> pos_read(eSVH_DIMENSION, false);
  // target positions read counter
  u_int8_t pos_read_counter = 0;

  int32_t index = 0;
  std::vector<std::string>::const_iterator joint_name;
  for (joint_name = input->name.begin(); joint_name != input->name.end(); ++joint_name,++index)
  {
    int32_t channel = 0;
    if (icl_core::string2Enum((*joint_name), channel, SVHController::m_channel_description))
    {
      if (input->position.size() > index)
      {
        target_positions[channel] = input->position[index];
        pos_read[channel] = true;
        pos_read_counter++;
      }
      else
      {
        ROS_WARN("Vector of input joint state is too small! Cannot acces elemnt nr %i", index);
      }
    }
    else
    {
      //ROS_WARN("Could not map joint name %s to channel!", (*joint_name).c_str());
    }
  }

  // send target values at once
  if (pos_read_counter == eSVH_DIMENSION)
  {
    if (!fm->setAllTargetPositions(target_positions))
    {
      ROS_WARN("Set target position command rejected!");
    }
  }
  // not all positions has been set: send only the available positions
  else
  {
    for (size_t i = 0; i < eSVH_DIMENSION; ++i)
    {
      if (pos_read[i])
      {
        fm->setTargetPosition(static_cast<SVHChannel>(i), target_positions[i], 0.0);
      }
    }
  }
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "svh_controller");
  ros::NodeHandle nh("~");

  icl_core::logging::initialize();

  // setting up dynamic reconfigure
  dynamic_reconfigure::Server<svh_controller::svhConfig> server;
  dynamic_reconfigure::Server<svh_controller::svhConfig>::CallbackType f;

  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);

  bool autostart;
  // ugly workaround, but ros param can not deal with the std::vector type ):
  bool disable_flags[9];
  std::vector<bool> disable_flags_vec(9, false);

  try
  {
    nh.param<bool>("autostart",autostart,false);

    nh.param<bool>("disable_flags0",disable_flags[0],false);
    nh.param<bool>("disable_flags1",disable_flags[1],false);
    nh.param<bool>("disable_flags2",disable_flags[2],false);
    nh.param<bool>("disable_flags3",disable_flags[3],false);
    nh.param<bool>("disable_flags4",disable_flags[4],false);
    nh.param<bool>("disable_flags5",disable_flags[5],false);
    nh.param<bool>("disable_flags6",disable_flags[6],false);
    nh.param<bool>("disable_flags7",disable_flags[7],false);
    nh.param<bool>("disable_flags8",disable_flags[8],false);

  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error! ");
  }

  for (size_t i = 0; i < 9; ++i)
  {
    // ugly workaround, but ros param can not deal with the std::vector type ):
    disable_flags_vec[i] = disable_flags[i];
    if(disable_flags[i])
    {
      ROS_WARN("svh_controller disabling channel nr %i", i);
    }
  }

  fm = boost::shared_ptr<SVHFingerManager>(new SVHFingerManager(autostart, disable_flags_vec));


  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, connectCallback);

  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub = nh.subscribe("reset_channel", 1, resetChannelCallback);

  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub = nh.subscribe("enable_channel", 1, enableChannelCallback);

  // Subscribe joint state topic
  ros::Subscriber channel_target_sub = nh.subscribe<sensor_msgs::JointState>("channel_targets", 1, jointStateCallback);

  // Publish current channel positions
  ros::Publisher channel_pos_pub = nh.advertise<sensor_msgs::JointState>("channel_feedback", 1);

  // joint state message template
  sensor_msgs::JointState channel_pos;
  channel_pos.name.resize(eSVH_DIMENSION);
  channel_pos.position.resize(eSVH_DIMENSION, 0.0);
  for (size_t channel = 0; channel < eSVH_DIMENSION; ++channel)
  {
    channel_pos.name[channel] = SVHController::m_channel_description[channel];
  }

  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(100);
  // Main loop.
  while (nh.ok())
  {
    if (fm->isConnected())
    {
      // Publish channel positions in RAD.
      for (size_t channel = 0; channel < eSVH_DIMENSION; ++channel)
      {
        double cur_pos = 0.0;
        fm->getPosition(static_cast<SVHChannel>(channel), cur_pos);

        channel_pos.position[channel] = cur_pos;
      }

      channel_pos_pub.publish(channel_pos);
    }
    ros::spinOnce();
    rate.sleep();
  }

  fm->disconnect();

  return 0;
} // end main()
