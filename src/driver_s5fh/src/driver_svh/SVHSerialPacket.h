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
 * \author  Georg Heppner
 * \date    2014-02-03
 * \date    2014-07-16
 *
 * This file contains the SVHSerialPacket data structure that is used to
 * send and receive everything from an to the actual hardware.
 * The serial packet is used to wrap up the payload data for convenient handling.
 * By wrapping everything in the same packet structure it can be handled quite neatly
 */
//----------------------------------------------------------------------
#ifndef SVHSERIALPACKET_H
#define SVHSERIALPACKET_H

#include <icl_comm/ByteOrderConversion.h>

namespace driver_svh {

//===============
// Some constants
//===============

// packet sizes
const size_t cPACKET_APPENDIX_SIZE = 8; //!< The packet overhead size in bytes
const size_t cDEFAULT_PACKET_SIZE = 48; //!< Default packet payload size in bytes

// packet headers
const uint8_t PACKET_HEADER1 = 0x4C;    //!< Header sync byte 1
const uint8_t PACKET_HEADER2 = 0xAA;    //!< Header sync byte 2

// adress constants for commands
const uint8_t SVH_GET_CONTROL_FEEDBACK      = 0x00; //!< Request the position and current of a channel to be sent
const uint8_t SVH_SET_CONTROL_COMMAND       = 0x01; //!< Sets the target position of a channel
const uint8_t SVH_GET_CONTROL_FEEDBACK_ALL  = 0x02; //!< Requests the positions and currents of ALL channels
const uint8_t SVH_SET_CONTROL_COMMAND_ALL   = 0x03; //!< Sends the target position to ALL the channels
const uint8_t SVH_GET_POSITION_SETTINGS     = 0x04; //!< Requests the active settings of the position controller
const uint8_t SVH_SET_POSITION_SETTINGS     = 0x05; //!< Sets new settings for the position controller
const uint8_t SVH_GET_CURRENT_SETTINGS      = 0x06; //!< Requests the active settings of the current controller
const uint8_t SVH_SET_CURRENT_SETTINGS      = 0x07; //!< Sets new settings for the current controller
const uint8_t SVH_GET_CONTROLLER_STATE      = 0x08; //!< Requests the state of the controller (active,faults,enabled channels)
const uint8_t SVH_SET_CONTROLLER_STATE      = 0x09; //!< Sets new controller states (enable channels, clear faults)
const uint8_t SVH_GET_ENCODER_VALUES        = 0x0A; //!< Request the current encoder scalings
const uint8_t SVH_SET_ENCODER_VALUES        = 0x0B; //!< Set new encoder scalings
const uint8_t SVH_GET_FIRMWARE_INFO         = 0x0C; //!< Request the firmware info to be transmitted

/*!
 * \brief The SerialPacket holds the (non generated) header and data of one message to the SVH-Hardware
 */
struct SVHSerialPacket
{
  //! \brief Continuosly incremented counter per package
  //! \note Continuous counter is currently not used by this software, the hw will just copy the send counter back to the response
  uint8_t index;
  //! Adress denotes the actual function of the package
  uint8_t address;
  //! Payload of the package
  std::vector <uint8_t> data;

  /*!
   * \brief SVHSerialPacket contains the send and received data in raw format (bytewise)
   * \param data_length initial size to set the data length to. NOTE: To deserialize a packet this value HAS TO BE SET!
   */
  SVHSerialPacket(size_t _data_length=0,uint8_t _address = SVH_GET_CONTROL_FEEDBACK):
    address(_address),
    data(_data_length,0)
  {
  }

  //! Compares two SVHSerialPackets objects.
  bool operator == (const SVHSerialPacket& other) const
  {
    return
      (index == other.index
       && address == other.address
       && data == other.data);
  }
};

//! overload stream operator to easily serialize raw packet data
icl_comm::ArrayBuilder& operator << (icl_comm::ArrayBuilder& ab, const SVHSerialPacket& data);

//! overload stream operator to easily deserialize raw packet data
icl_comm::ArrayBuilder& operator >> (icl_comm::ArrayBuilder& ab, SVHSerialPacket& data);

//! Output Stream operator for easy printing of packet data
std::ostream& operator << (std::ostream& o, const SVHSerialPacket& sp);

}
#endif // SVHSERIALPACKET_H
