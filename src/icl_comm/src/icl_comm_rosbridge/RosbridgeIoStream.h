// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jan Oberländer <oberlaen@fzi.de>
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2012-09-14
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_COMM_ROSBRIDGE_ROSBRIDGE_IO_STREAM_H_INCLUDED
#define ICL_COMM_ROSBRIDGE_ROSBRIDGE_IO_STREAM_H_INCLUDED

#include <icl_comm_tcp/TcpIoStream.h>
#include <json/json.h>

namespace icl_comm {
namespace rosbridge {

/*! An I/O stream for use with the ROSBRIDGE protocol.  Uses boost::asio
 *  internally.
 *  Main functionality is to strip away the protocol overhead. Everything is transmitted
 *  as JSON encoded message which should be parsed later on
 */
class RosbridgeIoStream : public icl_comm::tcp::TcpIoStream
{
public:
  /*! Constructs a stream.  Use connect() to open a connection.
   *  \param timeout The timeout, in milliseconds, when waiting for
   *         ROSBRIDGE telegram data.
   *  \param max_buffer_size The maximum size of the internal send and
   *         receive buffers.
   */
  RosbridgeIoStream(uint32_t timeout, std::size_t max_buffer_size=8192);

  /*! Reads any data type from the TCP connection.  On a timeout, the
   *  stream state goes bad and operator bool() returns \c false.
   */
  template <typename T>
  RosbridgeIoStream& operator >> (T& obj);

public:
  //! ASCII STX character (start of a ROSBRIDGE telegram).
  static const unsigned char cSTX;
  //! ASCII ETX character (end of a ROSBRIDGE telegram).
  static const unsigned char cETX;

protected:
  /*! Helper class which declares spaces, newlines and the '\x00' and '\xff'
   *  characters as stream delimiters.  This is used for parsing the
   *  ROSBRIDGE telegrams.
   */
  struct StreamDelimiters : std::ctype<char>
  {
    StreamDelimiters() : std::ctype<char>(get_table()) { }

    static std::ctype_base::mask const* get_table();
  };

  /*! Helper function which parses a Json Value of the stream
    */
  RosbridgeIoStream& parseValue(Json::Value& value);
};

}
}

#endif
