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
#include <icl_comm_serial/Serial.h>

using icl_comm::ArrayBuilder;
using icl_comm::serial::Serial;
using icl_comm::serial::SerialFlags;

// testing serial interface of svh driver
int main(int argc, const char* argv[])
{
  icl_core::logging::initialize();

  std::string serial_device_name = "/dev/ttyUSB1";

  Serial *serial_device = new Serial(serial_device_name.c_str(), SerialFlags(SerialFlags::eBR_921600, SerialFlags::eDB_8));
  serial_device->Open();

  uint8_t data = 0;
  while(true)
  {
    if (serial_device->Read(&data, sizeof(uint8_t)))
    {
      std::cout << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(data) << " " << std::flush;
    }
    else
    {
      std::cout << "." << std::flush;
    }
  }

  serial_device->Close();

}
