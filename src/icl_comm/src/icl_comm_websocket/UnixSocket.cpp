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
 * \author  Georg Heppner <heppner@fzi.de>
 * \author  Klaus Fischnaller <fischnal@fzi.de>
 * \date    2014-05-09
 *
 */
//----------------------------------------------------------------------
#include "UnixSocket.h"
#include <string.h>
#include <stdlib.h>

namespace icl_comm {
namespace websocket {

UnixSocket::UnixSocket(std::string sock_path) : path(sock_path)
{
  reconnect();
}

UnixSocket::~UnixSocket() {
  close(sock);
}

void UnixSocket::reconnect() {
  struct sockaddr_un remote;
  if (sock > 0)
  {
    close(sock);
  }

  // create socket
  if ((sock = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
  {
    throw SocketException("Could not open socket");
  }

  remote.sun_family = AF_UNIX;
  strcpy(remote.sun_path, path.c_str());
  int len = strlen(remote.sun_path) + sizeof(remote.sun_family);

  // connect to socket
  if (connect(sock, reinterpret_cast<sockaddr*>(&remote), len) == -1)
  {
    throw SocketException("Could not connect to socket");
  }
}

bool UnixSocket::send_raw_message(std::string msg)
{
  return (!(send(sock, msg.c_str(), msg.length(), 0) == -1));
}


}}
