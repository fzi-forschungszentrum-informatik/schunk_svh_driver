// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of FZIs ic_workspace.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2012-09-10
 *
 *
 */
//----------------------------------------------------------------------
#include "RosbridgeClient.h"
#include "icl_comm_rosbridge/Logging.h"

namespace icl_comm  {
namespace rosbridge {

RosbridgeClient::RosbridgeClient(uint32_t timeout)
  : m_host(),
    m_port(),
    m_stream(timeout)
{
}

bool RosbridgeClient::connect(const icl_core::String& host, const icl_core::String& port)
{
  m_host = host;
  m_port = port;
  LOGGING_DEBUG_CO(ROSBRIDGE, RosbridgeClient, m_host.c_str(),
                   "Connecting to " << m_host << ":" << m_port << "." << endl);
  try
  {
    m_stream.connect(m_host, m_port);
  }
  catch (...)
  {
    // FIXME: error handling
  }
  return m_stream.isOpen();
}

void RosbridgeClient::close()
{
  if (m_stream.isOpen())
    m_stream.close();
}

void RosbridgeClient::startRawMode()
{
  // Otherwise RosBridge expects a websocket connection
  m_stream << "raw\r\n\r\n" << std::flush;
}

void RosbridgeClient::sendJSon(const Json::Value& value)
{
  m_stream << RosbridgeIoStream::cSTX << m_json_writer.write(value) << RosbridgeIoStream::cETX << "\n" << std::flush;
}

void RosbridgeClient::sendDouble(const std::string& topic, const double& value)
{
  Json::Value message;
  Json::Value message_payload;

  // TODO: This can probably be put into one call... i am somehow unable to
  message_payload["data"] = value;

  message["receiver"] = topic;
  message["type"] = "std_msgs/Float64";
  message["msg"] = message_payload;

  //std::cout << "Sending Message: " << m_json_writer.write(message) << std::flush;
  sendJSon(message);
}

void RosbridgeClient::sendTwist(const std::string& topic, const double& linear_x,  const double& linear_y, const double& linear_z, const double& angular_x, const double& angular_y, const double& angular_z)
{
  Json::Value message;
  Json::Value message_payload_linear;
  Json::Value message_payload_angular;
  Json::Value message_payload;

  // TODO: This can probably be put into one call... i am somehow unable to
  message_payload_linear["x"] = linear_x;
  message_payload_linear["y"] = linear_y;
  message_payload_linear["z"] = linear_z;

  message_payload_angular["x"] = angular_x;
  message_payload_angular["y"] = angular_y;
  message_payload_angular["z"] = angular_z;

  message_payload["linear"] = message_payload_linear;
  message_payload["angular"] = message_payload_angular;

  message["receiver"] = topic;
  message["type"] = "geometry_msgs/Twist";
  message["msg"] = message_payload;

  //std::cout << "Sending Twist Message: " << m_json_writer.write(message) << std::flush;
  sendJSon(message);
}

void RosbridgeClient::sendJointStates(const std::string& topic, std::vector<std::string> name,
                                      const std::vector<double>& position,
                                      const std::vector<double>& velocity,
                                      const std::vector<double>& effort)
{
  Json::Value message;
  Json::Value message_payload_names(Json::arrayValue);
  Json::Value message_payload_positions(Json::arrayValue);
  Json::Value message_payload_velocities(Json::arrayValue);
  Json::Value message_payload_efforts(Json::arrayValue);
  Json::Value message_payload;

  if(position.size())
  {
    for(unsigned int i = 0; i<name.size(); i++)
    {
      message_payload_names.append(Json::Value(name[i]));
    }
    for(unsigned int i = 0; i<position.size(); i++)
    {
      message_payload_positions.append(Json::Value(position[i]));
    }
    if(velocity.size())
    {
      for(unsigned int i = 0; i<velocity.size(); i++)
      {
        message_payload_velocities.append(Json::Value(velocity[i]));
      }
    }
    if(effort.size())
    {
      for(unsigned int i = 0; i<effort.size(); i++)
      {
        message_payload_efforts.append(Json::Value(effort[i]));
      }
    }
  }

  message_payload["name"] = message_payload_names;
  message_payload["position"] = message_payload_positions;
  message_payload["velocity"] = message_payload_velocities;
  message_payload["effort"] = message_payload_efforts;

  message["receiver"] = topic;
  message["type"] = "sensor_msgs/JointState";
  message["msg"] = message_payload;

  std::cout << "Sending Joints Message: " << m_json_writer.write(message) << std::flush;
  sendJSon(message);
}

void RosbridgeClient::subscribeTopic(const std::string& topic, const std::string& msg_type)
{
  // Prepare the payload message ( same as above, you could probably do this in one call)
  Json::Value array;
  Json::Value message;

  // TODO: put this into one call somehow the syntax is strange with json...
  array[0] = topic;
  array[1] = 0; // -1= queque. 0 = as fast as possible, oher= time between messages, This could also be the Cycle time of the sensor call
  array[2] = msg_type;

  // Construct service call
  message["receiver"] = "/rosbridge/subscribe";
  message["msg"] = array;

  sendJSon(message);
}


std::list<Json::Value> RosbridgeClient::getAllMessages(uint32_t timeout)
{
  timeout = (timeout>10)?timeout-10:timeout; // Just substract a little to be finished for the next cycle
  icl_core::TimeStamp end = icl_core::TimeStamp::now() + icl_core::TimeSpan::createFromMSec(timeout);
  std::list<Json::Value> topics;
  Json::Value message;

  try
  {
    while (icl_core::TimeStamp::now() < end)
    {
      // Try to get every message
      message.clear();
      m_stream >> message;

      // Filter out answers (for example the OK Messages from service requests)
      if (message["msg"].isObject())
        topics.push_back(message);
    }

  } catch (const boost::system::errc::errc_t& e)
  {
    // In this case the Timeout just means that there are no messages left, this is okay and can be ignored
    // TODO: this function could give a return value to indicate if all messages could be read in time, as an indicator that the timeout is to short
  }
  return topics;
}

Json::Value RosbridgeClient::getMessage()
{
  Json::Value message;
  try
  {
    // Filter out answers (for example the OK Messages from service requests)
    do
    {
      message.clear();
      m_stream >> message;
    }while(!message["msg"].isObject());

  } catch (const boost::system::errc::errc_t& e)
  {
    // Single message timeout should be propagated
    throw;
  }
  //std::cout << "Received: " << m_json_writer.write(m_message) << "\n" << std::flush;
  return message;

}


}
}
