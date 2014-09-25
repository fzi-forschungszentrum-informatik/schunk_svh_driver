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
 * \date    2014-01-30
 * \date    2014-07-16
 *
 * This file contains the SVHSerialInterface class that is used to
 * handle the protocoll overhead of the serial communication.
 * It uses an icl_comm serial device that opens the physical connection and
 * is responsible to manage this hardware resource as well as protocoll issues
 * like sync bytes, checksum calculation and counting of packets send and received.
 */
//----------------------------------------------------------------------
#include "driver_svh/SVHSerialInterface.h"
#include "driver_svh/Logging.h"

#include <icl_comm/ByteOrderConversion.h>

using icl_core::TimeSpan;
using icl_comm::serial::SerialFlags;

namespace driver_svh {

SVHSerialInterface::SVHSerialInterface(ReceivedPacketCallback const & received_packet_callback) :
  m_connected(false),
  m_received_packet_callback(received_packet_callback),
  m_packets_transmitted(0)
{
}

SVHSerialInterface::~SVHSerialInterface()
{
  //close();
}

bool SVHSerialInterface::connect(const std::string &dev_name)
{
  // close device if already opened
  close();

  // create serial device
  m_serial_device.reset(new Serial(dev_name.c_str(), SerialFlags(SerialFlags::eBR_921600, SerialFlags::eDB_8)));

  if (m_serial_device)
  {
    // open serial device
    if (!m_serial_device->Open())
    {
      LOGGING_ERROR_C(DriverSVH, SVHSerialInterface, "Could not open serial device: " << dev_name.c_str() << endl);
      return false;
    }
  }
  else
  {
    LOGGING_ERROR_C(DriverSVH, SVHSerialInterface, "Could not create serial device handle: " << dev_name.c_str() << endl);
    return false;
  }

  // create receive thread
  m_receive_thread.reset(new SVHReceiveThread(TimeSpan(0, 500000), m_serial_device, m_received_packet_callback));

  if (m_receive_thread)
  {
    // start receive thread
    if (!m_receive_thread->start())
    {
      LOGGING_ERROR_C(DriverSVH, SVHSerialInterface, "Could not start the receive thread for the serial device!" << endl);
      return false;
    }
  }
  else
  {
    LOGGING_ERROR_C(DriverSVH, SVHSerialInterface, "Could not create the receive thread for the serial device!" << endl);
    return false;
  }

  m_connected = true;
  LOGGING_TRACE_C(DriverSVH, SVHSerialInterface, "Serial device  " << dev_name.c_str()  << " opened and receive thread started. Communication can now begin." << endl);

  return true;
}

void SVHSerialInterface::close()
{
  m_connected = false;

  // cancel and delete receive packet thread
  if (m_receive_thread)
  {
    // wait until thread has stopped
    m_receive_thread->stop();
    m_receive_thread->join();

    m_receive_thread.reset();

    LOGGING_TRACE_C(DriverSVH, SVHSerialInterface, "Serial device receive thread was terminated." << endl);
  }

  // close and delete serial device handler
  if (m_serial_device)
  {
    m_serial_device->Close();

    m_serial_device.reset();
    LOGGING_TRACE_C(DriverSVH, SVHSerialInterface, "Serial device handle was closed and terminated." << endl);
  }
}

bool SVHSerialInterface::sendPacket(SVHSerialPacket& packet)
{
  if (m_serial_device != NULL)
  {
    uint8_t check_sum1 = 0;
    uint8_t check_sum2 = 0;

    // Calculate Checksum for the packet
    for (size_t i = 0; i < packet.data.size(); i++)
    {
      check_sum1 += packet.data[i];
      check_sum2 ^= packet.data[i];
    }

    // set packet counter
    packet.index = static_cast<uint8_t>(m_packets_transmitted % uint8_t(-1));

    if (m_serial_device->IsOpen())
    {
      // Prepare arraybuilder
      size_t size = packet.data.size() + cPACKET_APPENDIX_SIZE;
      icl_comm::ArrayBuilder send_array(size);
      // Write header and packet information and checksum
      send_array << PACKET_HEADER1 << PACKET_HEADER2 << packet << check_sum1 << check_sum2;

      // actual hardware call to send the packet
      size_t bytes_send = 0;
      while (bytes_send < size)
      {
        bytes_send += m_serial_device->Write(send_array.array.data() + bytes_send, size - bytes_send);
      }

      // Small delay -> THIS SHOULD NOT BE NECESSARY as the communication speed should be handable by the HW. However, it will die if this sleep is
      // not used and this may also depend on your computer speed -> This issue might stem also from the hardware and will hopefully be fixed soon.
      icl_core::os::usleep(8000);

    }
    else
    {
      LOGGING_TRACE_C(DriverSVH, SVHSerialInterface, "sendPacket failed, serial device was not properly initialized." << endl);
      return false;
    }

    m_packets_transmitted++;
  }

  return true;
}

void SVHSerialInterface::resetTransmitPackageCount()
{
  m_packets_transmitted = 0;
}

void SVHSerialInterface::printPacketOnConsole(SVHSerialPacket &packet)
{

  uint8_t check_sum1 = 0;
  uint8_t check_sum2 = 0;

  // Calculate Checksum for the packet
  for (size_t i = 0; i < packet.data.size(); i++)
  {
    check_sum1 += packet.data[i];
    check_sum2 ^= packet.data[i];
  }

  // set packet counter
  packet.index = static_cast<uint8_t>(m_dummy_packets_printed % uint8_t(-1));


  // Prepare arraybuilder
  size_t size = packet.data.size() + cPACKET_APPENDIX_SIZE;
  icl_comm::ArrayBuilder send_array(size);
  // Write header and packet information and checksum
  send_array << PACKET_HEADER1 << PACKET_HEADER2 << packet << check_sum1 << check_sum2;

  std::cout << send_array << std::endl;

  m_dummy_packets_printed++;
}

}
