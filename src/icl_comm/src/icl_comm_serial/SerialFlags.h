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
 * \author  Lars Pfotzer <pfotzer@fzi.de>
 * \date    2013-03-08
 *
 * \author  Klaus Uhl
 * \date    2007-11-06
 *
 * \brief   Contains tSerialFlags
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_COMM_SERIAL_SERIALFLAGS_H_INCLUDED
#define ICL_COMM_SERIAL_SERIALFLAGS_H_INCLUDED

#ifdef _SYSTEM_WIN32_
#include <Windows.h>
#endif

#include "icl_comm_serial/ImportExport.h"

namespace icl_comm {
namespace serial {

//! Short description of tSerialFlags
/*! A more detailed description of tSerialFlags, which
    Klaus Uhl hasn't done till now!
*/
class ICL_COMM_SERIAL_IMPORT_EXPORT SerialFlags
{
public:

  enum DataBits
  {
    eDB_5 = 5,
    eDB_6 = 6,
    eDB_7 = 7,
    eDB_8 = 8
  };

  enum StopBits
  {
    eSB_1,
    eSB_1P5,
    eSB_2
  };

  enum Parity
  {
    eP_NONE,
    eP_EVEN,
    eP_ODD,
    eP_MARK,
    eP_SPACE
  };

  enum BaudRate
  {
    eBR_0 = 0,
    eBR_50 = 50,
    eBR_75 = 75,
    eBR_110 = 110,
    eBR_134 = 134,
    eBR_150 = 150,
    eBR_200 = 200,
    eBR_300 = 300,
    eBR_600 = 600,
    eBR_1200 = 1200,
    eBR_1800 = 1800,
    eBR_2400 = 2400,
    eBR_4800 = 4800,
    eBR_9600 = 9600,
    eBR_19200 = 19200,
    eBR_38400 = 38400,
    eBR_57600 = 57600,
    eBR_115200 = 115200,
    eBR_230400 = 230400,
    eBR_500000 = 500000,
    eBR_921600 = 921600
  };

  enum FlowControl
  {
    eFC_FLOW,
    eFC_HAND_SHAKE
  };

  enum ModemControlFlags
  {
    eMCF_NULL = 0x000,
    eMCF_DTR = 0x002,
    eMCF_RTS = 0x004,
    eMCF_UNDEFINED = 0x800
  };

  SerialFlags() :
    m_data_bits(eDB_8),
    m_stop_bits(eSB_1),
    m_parity(eP_NONE),
    m_baud_rate(eBR_9600),
    m_flow_control(eFC_HAND_SHAKE),
    m_use_modem_control(true),
    m_modem_control_flags(eMCF_UNDEFINED),
    m_enable_receiver(false),
    m_enable_stop_on_receive(false)
  {}

  SerialFlags(DataBits data_bits, Parity parity = eP_NONE, bool use_modem_control = false,
               bool enable_receiver = false, bool enable_stop_on_receive = false, ModemControlFlags modem_control_flags = eMCF_UNDEFINED) :
    m_data_bits(data_bits),
    m_stop_bits(eSB_1),
    m_parity(parity),
    m_baud_rate(eBR_9600),
    m_flow_control(eFC_HAND_SHAKE),
    m_use_modem_control(use_modem_control),
    m_modem_control_flags(modem_control_flags),
    m_enable_receiver(enable_receiver),
    m_enable_stop_on_receive(enable_stop_on_receive)
  {}

  SerialFlags(BaudRate baud_rate, DataBits data_bits, Parity parity = eP_NONE, bool use_modem_control = false,
               bool enable_receiver = false, bool enable_stop_on_receive = false, ModemControlFlags modem_control_flags = eMCF_UNDEFINED) :
    m_data_bits(data_bits),
    m_stop_bits(eSB_1),
    m_parity(parity),
    m_baud_rate(baud_rate),
    m_flow_control(eFC_HAND_SHAKE),
    m_use_modem_control(use_modem_control),
    m_modem_control_flags(modem_control_flags),
    m_enable_receiver(enable_receiver),
    m_enable_stop_on_receive(enable_stop_on_receive)
  {}

  SerialFlags(BaudRate baud_rate, Parity parity, DataBits data_bits, StopBits stop_bits,
               bool use_modem_control = false, bool enable_receiver = false, bool enable_stop_on_receive = false,
               ModemControlFlags modem_control_flags = eMCF_UNDEFINED) :
    m_data_bits(data_bits),
    m_stop_bits(stop_bits),
    m_parity(parity),
    m_baud_rate(baud_rate),
    m_flow_control(eFC_HAND_SHAKE),
    m_use_modem_control(use_modem_control),
    m_modem_control_flags(modem_control_flags),
    m_enable_receiver(enable_receiver),
    m_enable_stop_on_receive(enable_stop_on_receive)
  {}

  SerialFlags(const SerialFlags& flags) :
    m_data_bits(flags.m_data_bits),
    m_stop_bits(flags.m_stop_bits),
    m_parity(flags.m_parity),
    m_baud_rate(flags.m_baud_rate),
    m_flow_control(flags.m_flow_control),
    m_use_modem_control(flags.m_use_modem_control),
    m_modem_control_flags(flags.m_modem_control_flags),
    m_enable_receiver(flags.m_enable_receiver),
    m_enable_stop_on_receive(false)
  {}

  BaudRate getBaudRate() const { return m_baud_rate; }
  DataBits getDataBits() const { return m_data_bits; }
  bool enableStopOnReceive() const { return m_enable_stop_on_receive; }
  bool enableReceiver() const { return m_enable_receiver; }
  FlowControl getFlowControl() const { return m_flow_control; }
  Parity getParity() const { return m_parity; }
  StopBits getStopBits() const { return m_stop_bits; }
  bool useModemControl() const { return m_use_modem_control; }

  void setBaudRate(BaudRate baud_rate) { m_baud_rate = baud_rate; }

  ModemControlFlags getModemControlFlags()const { return m_modem_control_flags; }

#ifdef _SYSTEM_POSIX_
  unsigned long CFlags() const;
  static unsigned long CFlags(BaudRate baud_rate);
#endif
#ifdef _SYSTEM_WIN32_
  void GetDCB(LPDCB dcb) const;
#endif

private:
  DataBits m_data_bits;
  StopBits m_stop_bits;
  Parity m_parity;
  BaudRate m_baud_rate;
  FlowControl m_flow_control;
  bool m_use_modem_control;
  ModemControlFlags m_modem_control_flags;
  bool m_enable_receiver;
  bool m_enable_stop_on_receive;

};

}
}

#endif
