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
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2010-06-16
 *
 */
//----------------------------------------------------------------------
#ifndef ULS_UDP_LOGGING_SERVER_H_INCLUDED
#define ULS_UDP_LOGGING_SERVER_H_INCLUDED

#include <QtCore/QObject>
#include <QtNetwork/QUdpSocket>
#include <QtSql/QSqlDatabase>

class UdpLoggingServer : QObject
{
  Q_OBJECT

  public:
  UdpLoggingServer(const QString& db_filename);
  virtual ~UdpLoggingServer();

private slots:
  void processPendingDatagrams();

private:
  QUdpSocket *m_udp_socket;
  QSqlDatabase m_db;
};

#endif
