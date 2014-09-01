// ROS includes.
#include <ros/ros.h>

// Custom includes
#include "SVHNode.h"
#include <icl_core/EnumHelper.h>



/*--------------------------------------------------------------------
 * Callback functions
 *------------------------------------------------------------------*/

SVHNode::SVHNode(const bool &autostart,const std::vector<bool> & disable_flags_vec)
{
  // Initialize the icl_library logging framework
  icl_core::logging::initialize();

  serial_device_name_ = "";

  // prepare the channel position message for later sending
  channel_pos_.name.resize(driver_svh::eSVH_DIMENSION);
  channel_pos_.position.resize(driver_svh::eSVH_DIMENSION, 0.0);
  for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
  {
    channel_pos_.name[channel] = driver_svh::SVHController::m_channel_description[channel];
  }

  // Init the actual driver hook (after logging initialize)
  fm_.reset(new driver_svh::SVHFingerManager(autostart, disable_flags_vec));
}

SVHNode::~SVHNode()
{
  // Tell the driver to close connections
  fm_->disconnect();
}

// Callback function for changing parameters dynamically
void SVHNode::dynamic_reconfigure_callback(svh_controller::svhConfig &config, uint32_t level)
{
  serial_device_name_ = config.serial_device;
}

// Callback function for connecting to SCHUNK five finger hand
void SVHNode::connectCallback(const std_msgs::Empty&)
{
  if (fm_->isConnected())
  {
    fm_->disconnect();
  }

  if (!fm_->connect(serial_device_name_))
  {
    ROS_ERROR("Could not connect to SCHUNK five finger hand with serial device %s", serial_device_name_.c_str());
  }
}

// Callback function to reset/home channels of SCHUNK five finger hand
void SVHNode::resetChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  // convert int8 channel into SVHChannel enum
  driver_svh::SVHChannel svh_channel = static_cast<driver_svh::SVHChannel>(channel->data);

  if (fm_->resetChannel(svh_channel))
  {
    ROS_INFO("Channel %i successfully homed!", svh_channel);
  }
  else
  {
    ROS_ERROR("Could not reset channel %i !", svh_channel);
  }
}

// Callback function to enable channels of SCHUNK five finger hand
void SVHNode::enableChannelCallback(const std_msgs::Int8ConstPtr& channel)
{
  fm_->enableChannel(static_cast<driver_svh::SVHChannel>(channel->data));
}

// Callback function for setting channel target positions to SCHUNK five finger hand
void SVHNode::jointStateCallback(const sensor_msgs::JointStateConstPtr& input )
{
  // vector to read target positions from joint states
  std::vector<double> target_positions(driver_svh::eSVH_DIMENSION, 0.0);
  // bool vector storing true, if new target position read
  std::vector<bool> pos_read(driver_svh::eSVH_DIMENSION, false);
  // target positions read counter
  u_int8_t pos_read_counter = 0;

  int32_t index = 0;
  std::vector<std::string>::const_iterator joint_name;
  for (joint_name = input->name.begin(); joint_name != input->name.end(); ++joint_name,++index)
  {
    int32_t channel = 0;
    if (icl_core::string2Enum((*joint_name), channel, driver_svh::SVHController::m_channel_description))
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
  if (pos_read_counter == driver_svh::eSVH_DIMENSION)
  {
    if (!fm_->setAllTargetPositions(target_positions))
    {
      ROS_WARN("Set target position command rejected!");
    }
  }
  // not all positions has been set: send only the available positions
  else
  {
    for (size_t i = 0; i < driver_svh::eSVH_DIMENSION; ++i)
    {
      if (pos_read[i])
      {
        fm_->setTargetPosition(static_cast<driver_svh::SVHChannel>(i), target_positions[i], 0.0);
      }
    }
  }
}


sensor_msgs::JointState SVHNode::getCurrentPositions()
{
  if (fm_->isConnected())
  {
    // Get positions in rad
    for (size_t channel = 0; channel < driver_svh::eSVH_DIMENSION; ++channel)
    {
      double cur_pos = 0.0;
      if (fm_->isHomed(static_cast<driver_svh::SVHChannel>(channel)))
      {
        fm_->getPosition(static_cast<driver_svh::SVHChannel>(channel), cur_pos);
      }

      channel_pos_.position[channel] = cur_pos;
    }
  }

  return  channel_pos_;
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  //==========
  // ROS
  //==========

  // Set up ROS.
  ros::init(argc, argv, "svh_controller");
  // Private NH for params
  ros::NodeHandle nh("~");

  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(100);

  //==========
  // Params
  //==========

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

    // TODO: What About the Serial Device Name? Rad up on dynamic reconfigure first

  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error!");
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


  //==========
  // Logic
  //==========
  // Node object holding all the relevant functions
  SVHNode svh_node(autostart,disable_flags_vec);

  //==========
  // Dynamic Reconfigure
  //==========

  dynamic_reconfigure::Server<svh_controller::svhConfig> server;
  dynamic_reconfigure::Server<svh_controller::svhConfig>::CallbackType f;

  f = boost::bind(&SVHNode::dynamic_reconfigure_callback,&svh_node, _1, _2);
  server.setCallback(f);

  //==========
  // Callbacks
  //==========

  // Subscribe connect topic (Empty)
  ros::Subscriber connect_sub = nh.subscribe("connect", 1, &SVHNode::connectCallback,&svh_node);
  // Subscribe reset channel topic (Int8)
  ros::Subscriber reset_sub = nh.subscribe("reset_channel", 1, &SVHNode::resetChannelCallback,&svh_node);
  // Subscribe enable channel topic (Int8)
  ros::Subscriber enable_sub = nh.subscribe("enable_channel", 1, &SVHNode::enableChannelCallback, &svh_node);
  // Subscribe joint state topic
  ros::Subscriber channel_target_sub = nh.subscribe<sensor_msgs::JointState>("channel_targets", 1, &SVHNode::jointStateCallback,&svh_node );
  // Publish current channel positions
  ros::Publisher channel_pos_pub = nh.advertise<sensor_msgs::JointState>("channel_feedback", 1);

  //==========
  // Messaging
  //==========

  // Main loop.
  while (nh.ok())
  {
    // get the current positions of all joints and publish them
    channel_pos_pub.publish(svh_node.getCurrentPositions());

    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
