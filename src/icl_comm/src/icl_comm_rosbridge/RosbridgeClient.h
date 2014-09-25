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
 * \author  Jan Oberländer <oberlaen@fzi.de>
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2012-09-10
 * stole lots of the code from icl_hardware_sopasclient
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_COMM_ROSBRIDGE_CLIENT_H_INCLUDED
#define ICL_COMM_ROSBRIDGE_CLIENT_H_INCLUDED

#include <icl_core/BaseTypes.h>

#include <json/json.h>
#include <json/writer.h>


#include "icl_comm_rosbridge/Logging.h"
#include "icl_comm_rosbridge/RosbridgeIoStream.h"

namespace icl_comm {
namespace rosbridge {

/*! A client for using the rosbridge Protocol via TCP
 *  A client for using the rosbridge Protocol via TCP   All
 *  functions except the getAllMessages may throw a
 *  boost::system::errc::timed_out exception if
 *  an expected rosbridge reply was not received within the timeout
 *  specified in the constructor.
 */
class RosbridgeClient
{
public:
  /*! Create a new ROSBRIDGE client.
   *  \param timeout The time, in milliseconds, after which a timeout
   *         exception (boost::system::errc::timed_out) is raised. This timeout is used in the getAll messages to detect an empty message stream
   */
  explicit RosbridgeClient(uint32_t timeout = 10);

  /*! Connect to a ROSBRIDGE server.
   *  \param host The hostname of the ROSBRIDGE server.
   *  \param port The TCP port of the ROSBRIDGE server.  Given as a
   *         string, so that a service name can be specified as well.
   */
  bool connect(const icl_core::String& host, const icl_core::String& port = "9090");

  //! Returns \c true if a connection is open.
  bool isOpen() const { return m_stream.isOpen(); }

  //! Closes the connection.
  void close();

  //! Returns \c true if the connection is OK.
  operator bool () const { return m_stream; }

  //! Opens the Socket as a raw TCP Socket
  void startRawMode();

  //! Returns a single JSON message
  Json::Value getMessage();

  //! Gets all the JSON Messages that are there, Stops collecting messages after a predefined timeout
  /*
   * \param timout in msec after which the reading of messages should be terminated (this is intended for cycled read where the function needs to terminate bevore the next cycle)
   * \return list of JSON objects
   */
  std::list<Json::Value> getAllMessages(uint32_t timeout);

  /*! Subscribes to a topic on the ROS side
   *  After subscribing a JSON message with the result of the operation will be sent back.
   *  This message is currently filtered out completely.
   * \param topic the topic to subscribe to
   * \param, msg_ytype the msg_type to advertise for the ROS Topic
   */
  void subscribeTopic(const std::string& topic,const std::string& msg_type);

  /*! Send out a double value to a specific topic. Double will be advertised as std_msgs/Float64
   *\param topic the ROSTopic to advertise on (will be created if not existent)
   *\param value the double value to be sent
   *
   */
  void sendDouble(const std::string& topic,const double& value);

  /*! Send out a twist message to a specific topic. Type is 'geometry_msgs/Twist'.
   *\param topic the ROSTopic to advertise on (will be created if not existent)
   *\param linear_x a single value of the twist message data to be sent
   *\param linear_y a single value of the twist message data to be sent
   *\param linear_z a single value of the twist message data to be sent
   *\param angular_x a single value of the twist message data to be sent
   *\param angular_y a single value of the twist message data to be sent
   *\param angular_z a single value of the twist message data to be sent
   *
   */
  void sendTwist(const std::string& topic, const double& linear_x,  const double& linear_y, const double& linear_z, const double& angular_x, const double& angular_y, const double& angular_z);

  /* Sends out a joint_state message.
   * the Params after position are optional. But all vectors have to be of the same size,
   * if used or empty else.
   */
  void sendJointStates(const std::string& topic,
                       const std::vector<std::string> name,
                       const std::vector<double>& position,
                       const std::vector<double>& velocity,
                       const std::vector<double>& effort);

private:
  //! Host name to connect to.
  icl_core::String m_host;
  //! Port to connect to.
  icl_core::String m_port;
  //! The ROSBRIDGE communication stream.
  RosbridgeIoStream m_stream;

  //! The JSon Value Writer
  Json::FastWriter m_json_writer;
  //! And value Reader
  Json::Reader m_json_reader;


  /*! Send a JSon encoded message to the server
   * \param value the message GIVEN in JSON Format.
   * @note: The values need to have the right structure, Therefore this is not public at the moment,
   * if other data formats are to be used then another wrapper is needed
   */
  void sendJSon(const Json::Value& value);



};

}
}

#endif
