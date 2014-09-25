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
#include <driver_svh/SVHPositionSettings.h>

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

  // build serial packet for sending
  ArrayBuilder payload(40);
  SVHSerialPacket test_serial_packet;
  SVHPositionSettings test_pos_settings(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1);

  // Get the Position settings as payload
  payload << test_pos_settings;
  // Generate the header information
  test_serial_packet.address = 5; // Set Position settings
  test_serial_packet.index = 0;   //
  // Set the payload (converted array of position settings)
  test_serial_packet.data = payload.array;

  // send packet via serial port
  serial_com.sendPacket(test_serial_packet);

  serial_com.close();
}
