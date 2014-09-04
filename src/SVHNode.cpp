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

// Callback function for changing parameters dynamically
void SVHNode::dynamic_reconfigure_callback_finger(svh_controller::svhFingerConfig &config, uint32_t level,const uint32_t & finger)
{
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

  channel_pos_.header.stamp = ros::Time::now();
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
  // Private NH for general params
  ros::NodeHandle nh("~");
  // private NH for specific finger params
  // This is required in order to easily differentiate between the parameter sets for the fingers AND
  // use dynamic reconfigure at the same time. Just Rosparams would be able to find the params within
  // sub namespaces, however this functionality is (to my knowledge) not yet implemented in Dynamic Reconfigure
  // To still provide a way of easy parameter update during runtime this seems to be the best way
  ros::NodeHandle nh_finger0("~/THUMB_FLEXION");
  ros::NodeHandle nh_finger1("~/THUMB_OPPOSITION");
  ros::NodeHandle nh_finger2("~/INDEX_FINGER_DISTAL");
  ros::NodeHandle nh_finger3("~/INDEX_FINGER_PROXIMAL");
  ros::NodeHandle nh_finger4("~/MIDDLE_FINGER_DISTAL");
  ros::NodeHandle nh_finger5("~/MIDDLE_FINGER_PROXIMAL");
  ros::NodeHandle nh_finger6("~/RING_FINGER");
  ros::NodeHandle nh_finger7("~/PINKY");
  ros::NodeHandle nh_finger8("~/FINGER_SPREAD");



  // Tell ROS how fast to run this node. (100 = 100 Hz = 10 ms)
  ros::Rate rate(100);

  //==========
  // Params
  //==========

  bool autostart;
  std::vector<bool> disable_flags(9, false);

  try
  {
    nh.param<bool>("autostart",autostart,false);
    // Note : Wrong values (like numerics) in the launch file will lead to a "true" value here
    nh.getParam("disable_flags",disable_flags);
    // Device and other params are used in dynamic reconfigure and need not to be handled individually
  }
  catch (ros::InvalidNameException e)
  {
    ROS_ERROR("Parameter Error!");
  }

  for (size_t i = 0; i < 9; ++i)
  {
    if(disable_flags[i])
    {
      ROS_WARN("svh_controller disabling channel nr %i", i);
    }
  }


  //==========
  // Logic
  //==========
  // Node object holding all the relevant functions
  SVHNode svh_node(autostart,disable_flags);

  //==========
  // Dynamic Reconfigure
  //==========

  dynamic_reconfigure::Server<svh_controller::svhConfig> server;
  // Same reason as stated above to have multiple reconfigure server:
  // If we would just use one, each parameter would need a unique name as the dynamic reconfigure
  // can not handle namespaces (at least to my knowledge) so we would need to define very many unique variables
  // By using this way we can listen to seperate namespaces. To provide the correct feedback within the node
  // we rely on constants in the boost binding which enables us to find out which namespace the reconfigure call came from
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger0(nh_finger0);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger1(nh_finger1);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger2(nh_finger2);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger3(nh_finger3);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger4(nh_finger4);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger5(nh_finger5);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger6(nh_finger6);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger7(nh_finger7);
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig> server_finger8(nh_finger8);

  dynamic_reconfigure::Server<svh_controller::svhConfig>::CallbackType f;
  dynamic_reconfigure::Server<svh_controller::svhFingerConfig>::CallbackType fi0,fi1,fi2,fi3,fi4,fi5,fi6,fi7,fi8;

  f = boost::bind(&SVHNode::dynamic_reconfigure_callback,&svh_node, _1, _2);
  fi0 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 0);
  fi1 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 1);
  fi2 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 2);
  fi3 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 3);
  fi4 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 4);
  fi5 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 5);
  fi6 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 6);
  fi7 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 7);
  fi8 = boost::bind(&SVHNode::dynamic_reconfigure_callback_finger,&svh_node, _1, _2, 8);

  server.setCallback(f);
  server_finger0.setCallback(fi0);
  server_finger1.setCallback(fi1);
  server_finger2.setCallback(fi2);
  server_finger3.setCallback(fi3);
  server_finger4.setCallback(fi4);
  server_finger5.setCallback(fi5);
  server_finger6.setCallback(fi6);
  server_finger7.setCallback(fi7);
  server_finger8.setCallback(fi8);


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
