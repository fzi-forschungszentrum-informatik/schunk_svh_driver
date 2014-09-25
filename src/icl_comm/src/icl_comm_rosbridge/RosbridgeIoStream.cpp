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
 * \author Georg Heppner <heppner@fzi.de>
 * \date    2012-09-14
 *
 */
//----------------------------------------------------------------------

#include "RosbridgeIoStream.h"

namespace icl_comm {
namespace rosbridge {

const unsigned char RosbridgeIoStream::cSTX = 0x00;
const unsigned char RosbridgeIoStream::cETX = 0xff;

std::ctype_base::mask const* RosbridgeIoStream::StreamDelimiters::get_table()
{

  static std::vector<std::ctype_base::mask> rc(table_size, std::ctype_base::mask());
  rc[' ']  = std::ctype_base::space;
  rc['\n'] = std::ctype_base::space;
  rc[cSTX] = std::ctype_base::space;
  rc[cETX] = std::ctype_base::space;
  return &rc[0];
}

RosbridgeIoStream::RosbridgeIoStream(uint32_t timeout, std::size_t max_buffer_size)
  : icl_comm::tcp::TcpIoStream(timeout, max_buffer_size)
{
  // This accepts STX and ETX characters as whitespace.
  imbue(std::locale(std::locale(), new StreamDelimiters()));
}

template <>
RosbridgeIoStream& RosbridgeIoStream::operator >> <Json::Value>(Json::Value& obj)
{
  parseValue(obj);
  return *this;
}


RosbridgeIoStream& RosbridgeIoStream::parseValue(Json::Value& value)
{
  // Make sure we get to read a whole word.
  while (!checkForDelimiter())
  {
    readMoreData();
  }
  // Skip initial spaces.
  while (!m_istream.eof() && std::isspace(char(m_istream.peek()), m_istream.getloc()))
  {
    m_istream.get();
  }
  // Get numeric value.
  if (m_istream.eof())
  {
    value = 0;
  }
  else
  {
      m_istream >> value;
  }
  return *this;
}


}
}
