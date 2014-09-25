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
 * \author  Jan Oberlaender
 * \date    2012-01-24
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_COMM_INTERFACE_ADDRESS_H_INCLUDED
#define ICL_COMM_INTERFACE_ADDRESS_H_INCLUDED

#include <boost/asio/ip/address.hpp>
#include <icl_core/BaseTypes.h>
#include <icl_core/Multimap.h>

#include "icl_comm/ImportExport.h"

namespace icl_comm {

struct ICL_COMM_IMPORT_EXPORT InterfaceAddress
{
  //! Constructor.
  InterfaceAddress(const boost::asio::ip::address& interface_address = boost::asio::ip::address(),
                   const boost::asio::ip::address& interface_netmask = boost::asio::ip::address());

  //! Interface address.
  boost::asio::ip::address interface_address;
  //! Interface netmask.
  boost::asio::ip::address interface_netmask;
};

//! Contains interface addresses sorted by interface name.
typedef icl_core::Multimap<icl_core::String, InterfaceAddress> InterfaceAddressMap;

extern InterfaceAddressMap getInterfaceAddresses(bool ipv6_support = false);

}

#endif
