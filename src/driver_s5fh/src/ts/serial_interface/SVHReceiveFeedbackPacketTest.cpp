// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \date    2014-02-15
 *
 */
//----------------------------------------------------------------------

#include <icl_comm/ByteOrderConversion.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHControllerFeedback.h>
#include <driver_svh/SVHController.h>
#include <driver_svh/SVHSerialPacket.h>
#include <boost/bind/bind.hpp>

using icl_comm::ArrayBuilder;
using namespace driver_svh;

void receivedPacketCallback(const SVHSerialPacket& packet, unsigned int packet_count)
{
  std::cout << "Received new packet with number " << packet_count << std::endl;

  // Extract Channel
  uint8_t channel = (packet.address >> 4 ) & 0x0F;
  // Prepare Data for conversion
  ArrayBuilder ab;
  ab.appendWithoutConversion(packet.data);

  std::cout << "channel = " << static_cast<int>(channel) << std::endl;

  if ((packet.address & 0x0F) == SVH_SET_CONTROL_COMMAND) // || ((packet.address & 0x0F) == SVH_SET_CONTROL_COMMAND)
  {
    SVHControllerFeedback controller_feedback;

    ab >> controller_feedback;
    LOGGING_INFO_C(DriverSVH, SVHController, "Received a Control Feedback/Control Command packet for channel "<< channel << endl);

    std::cout << "Controller Feedback " << controller_feedback << endl;
  }
}

// testing serial interface of svh driver
int main(int argc, const char* argv[])
{
  icl_core::logging::initialize();

  std::string serial_device_name = "/dev/ttyUSB2";

  SVHSerialInterface serial_com(boost::bind(&receivedPacketCallback,_1,_2));
  serial_com.connect(serial_device_name);

  while (true)
  { }

  serial_com.close();
}

