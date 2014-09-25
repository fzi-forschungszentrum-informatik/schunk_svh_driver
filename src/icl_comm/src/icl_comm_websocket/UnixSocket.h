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
#ifndef ICL_COMM_WEBSOCKET_UNIX_SOCKET_H_INCLUDED
#define ICL_COMM_WEBSOCKET_UNIX_SOCKET_H_INCLUDED


#include <unistd.h>
//#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
//#include <stdio.h>
//#include <stdint.h>
//#include <string>

#include <stdexcept>

namespace icl_comm {
namespace websocket {

class SocketException : public std::runtime_error {
public:
  SocketException(const std::string& arg) : std::runtime_error("Socket Exception: "+arg) { }
};

/*!
 * \brief The UnixSocket class sends messages via Unix Socket
 *
 * Let's you send messages to a server.
 * Uses UNIX Sockets to communicate.
 */
class UnixSocket
{
public:
    /*!
     * \brief UnixSocket Default ctor
     * \param sock_path path to unix socket
     */
    UnixSocket(std::string sock_path);

    /*!
     * \brief wsbroadcaster destructor
     *
     * Closes socket connection
     */
    ~UnixSocket();


    /*!
     * \brief Reconnects to socket at path
     */
    void reconnect();

    /*!
     * \brief Sends a message to the broadcasting server
     * \param msg the message to be sent
     */
    bool send_raw_message(std::string msg);
    
private:
    int           sock;
    size_t        size;
    std::string   path;
};

}}

#endif
