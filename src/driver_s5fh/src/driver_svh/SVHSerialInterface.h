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
 * \author  Lars pfotzer
 * \author  Georg Heppner
 * \date    2014-01-30
 * \date    2014-07-16
 *
 * This file contains the SVHSerialInterface class that is used to
 * handle the protocol overhead of the serial communication.
 * It uses an icl_comm serial device that opens the physical connection and
 * is responsible to manage this hardware resource as well as protocoll issues
 * like sync bytes, checksum calculation and counting of packets sent and received.
 */
//----------------------------------------------------------------------
#ifndef DRIVER_SVH_SVH_SERIAL_INTERFACE_H_INCLUDED
#define DRIVER_SVH_SVH_SERIAL_INTERFACE_H_INCLUDED

// Windows declarations
#include <driver_svh/ImportExport.h>

#include <driver_svh/SVHSerialPacket.h>
#include <driver_svh/SVHReceiveThread.h>
#include <icl_comm_serial/Serial.h>
#include <boost/shared_ptr.hpp>
using icl_comm::serial::Serial;

namespace driver_svh {

/*!
 *  \brief Basic communication handler for the SCHUNK five finger hand.
 */
class DRIVER_SVH_IMPORT_EXPORT SVHSerialInterface
{
public:
  //!
  //! \brief Constructs a serial interface class for basic communication with the SCHUNK five finger hand.
  //! \param received_packet_callback function to call whenever a packet was received
  //!
  SVHSerialInterface(const ReceivedPacketCallback &received_packet_callback);

  //! Default DTOR
  ~SVHSerialInterface();

  //!
  //! \brief connecting to serial device and starting receive thread
  //! \param dev_name Filehandle of the device i.e. dev/ttyUSB0
  //! \return bool true if connection was succesfull
  //!
  bool connect(const std::string &dev_name);

  //!
  //! \brief canceling receive thread and closing connection to serial port
  //!
  void close();

  //!
  //! \brief returns connected state of serial device
  //! \return bool true if the device is connected
  //!
  bool isConnected() { return m_connected; }

  //!
  //! \brief function for sending packets via serial device to the SVH
  //! \param packet the prepared Serial Packet
  //! \return true if successful
  //!
  bool sendPacket(SVHSerialPacket &packet);

  //!
  //! \brief get number of transmitted packets
  //! \return number of successfully sent packets
  //!
  unsigned int transmittedPacketCount() { return m_packets_transmitted; }

  /*!
   * \brief resetTransmitPackageCount Resets the transmitpackage count to zero
   */
  void resetTransmitPackageCount();

  /*!
   * \brief printPacketOnConsole is a pure helper function to show what raw data is actually sent. This is not meant for any productive use other than understand whats going on.
   * \param packet the prepared Serial Packet(without header information and such)
   */
  void printPacketOnConsole(SVHSerialPacket &packet);

private:

  //! serial device connected state
  bool m_connected;

  //! pointer to serial interface object
  boost::shared_ptr<Serial> m_serial_device;

  //! cecksum calculation
  void calcCheckSum(uint8_t &check_sum1, uint8_t &check_sum2, const SVHSerialPacket& packet);

  //! thread for receiving serial packets
  boost::shared_ptr<SVHReceiveThread> m_receive_thread;

  //! Callback function for received packets
  ReceivedPacketCallback m_received_packet_callback;

  //! packet counters
  unsigned int m_packets_transmitted;

  //! packet counter simulation for pure showing purposes
  unsigned int m_dummy_packets_printed;
};

}

#endif
