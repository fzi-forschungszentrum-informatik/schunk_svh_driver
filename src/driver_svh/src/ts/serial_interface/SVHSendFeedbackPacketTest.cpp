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
#include <driver_svh/SVHSerialInterface.h>
#include <driver_svh/SVHControllerFeedback.h>
#include <driver_svh/SVHController.h>
#include <driver_svh/SVHSerialPacket.h>

using icl_comm::ArrayBuilder;
using namespace driver_svh;

using icl_comm::serial::Serial;
using icl_comm::serial::SerialFlags;

// testing serial interface of svh driver
int main(int argc, const char* argv[])
{
  icl_core::logging::initialize();

  std::string serial_device_name = "/dev/ttyUSB0";

  SVHSerialInterface serial_com(NULL);
  serial_com.connect(serial_device_name);

  // build feedback serial packet for sending
  ArrayBuilder packet;
  SVHChannel channel = eSVH_PINKY;
  SVHSerialPacket test_serial_packet(40,SVH_SET_CONTROL_COMMAND|static_cast<uint8_t>(channel << 4));
  SVHControllerFeedback test_controller_feedback(0, 140);

  // serialize test controller feedback to paket
  packet << test_controller_feedback;
  test_serial_packet.index = 0;   //
  // Set the payload (converted array of position settings)
  test_serial_packet.data = packet.array;

  // send packet via serial port
  serial_com.sendPacket(test_serial_packet);

  icl_core::os::sleep(5);

  test_controller_feedback.position = -8000;

  // serialize test controller feedback to paket
  packet.reset(0);
  packet << test_controller_feedback;
  test_serial_packet.index = 0;   //
  // Set the payload (converted array of position settings)
  test_serial_packet.data = packet.array;

  // send packet via serial port
  serial_com.sendPacket(test_serial_packet);

  serial_com.close();
}

