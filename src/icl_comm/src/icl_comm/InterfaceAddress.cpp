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
#include "icl_comm/InterfaceAddress.h"

#ifdef _SYSTEM_POSIX_
# include <sys/types.h>
# include <ifaddrs.h>
# include <netinet/in.h>
# include <arpa/inet.h>
#else
# error "No implementation available yet for non-POSIX systems!"
#endif

namespace icl_comm {

InterfaceAddress::InterfaceAddress(const boost::asio::ip::address& interface_address,
                                   const boost::asio::ip::address& interface_netmask)
  : interface_address(interface_address),
    interface_netmask(interface_netmask)
{ }

InterfaceAddressMap getInterfaceAddresses(bool ipv6_support)
{
  using namespace boost::asio;
  InterfaceAddressMap result;

#ifdef _SYSTEM_POSIX_
  //--------------------------------------------------------------------
  // POSIX Implementation
  //--------------------------------------------------------------------
  struct ifaddrs *if_addresses = NULL;

  getifaddrs(&if_addresses);
  for (struct ifaddrs *iter = if_addresses; iter != NULL; iter = iter->ifa_next)
  {
    if (iter->ifa_addr->sa_family == AF_INET) // IPv4
    {
      ip::address_v4 addr(reinterpret_cast<const ip::address_v4::bytes_type&>(reinterpret_cast<struct sockaddr_in *>(iter->ifa_addr)->sin_addr.s_addr));
      ip::address_v4 netmask(reinterpret_cast<const ip::address_v4::bytes_type&>(reinterpret_cast<struct sockaddr_in *>(iter->ifa_netmask)->sin_addr.s_addr));
      result.insert(std::make_pair(icl_core::String(iter->ifa_name),
                                   InterfaceAddress(addr, netmask)));
    }
    else if (ipv6_support && iter->ifa_addr->sa_family == AF_INET6) // IPv6
    {
      ip::address_v6 addr(reinterpret_cast<const ip::address_v6::bytes_type&>(reinterpret_cast<struct sockaddr_in6 *>(iter->ifa_addr)->sin6_addr.s6_addr));
      ip::address_v6 netmask(reinterpret_cast<const ip::address_v6::bytes_type&>(reinterpret_cast<struct sockaddr_in6 *>(iter->ifa_netmask)->sin6_addr.s6_addr));
      result.insert(std::make_pair(icl_core::String(iter->ifa_name),
                                   InterfaceAddress(addr, netmask)));
    }
  }
  if (if_addresses)
  {
    freeifaddrs(if_addresses);
  }
  return result;
#endif
}

}
