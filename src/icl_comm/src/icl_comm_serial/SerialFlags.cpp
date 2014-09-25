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
 */
//----------------------------------------------------------------------

#include "icl_comm_serial/SerialFlags.h"

// Terminal headers are included after all Debug headers, because the
// Debug headers may include Eigen/Core from the Eigen matrix library.
// For Eigen3 there is a name clash with B0 from termio.h, and Eigen3
// silently #undefs it.  Therefore we make sure here that termio.h
// gets re-included last.
#ifdef _SYSTEM_POSIX_
# include <termios.h>
#ifdef _SYSTEM_DARWIN_
#define B500000 500000
#define B921600 921600
#endif
#endif

namespace icl_comm  {
namespace serial {

  //----------------------------------------------------------------------
  // SerialFlags
  //----------------------------------------------------------------------

  #ifdef _SYSTEM_POSIX_

  unsigned long SerialFlags::CFlags() const
  {
    unsigned long cflags = 0;

    switch (m_data_bits)
    {
      case eDB_5:
      {
        cflags |= CS5;
        break;
      }
      case eDB_6:
      {
        cflags |= CS6;
        break;
      }
      case eDB_7:
      {
        cflags |= CS7;
        break;
      }
      case eDB_8:
      {
        cflags |= CS8;
        break;
      }
    }

    if (m_stop_bits == eSB_2)
    {
      cflags |= CSTOPB;
    }

    if (m_parity != eP_NONE)
    {
      cflags |= PARENB;
    }

    if (m_parity == eP_ODD)
    {
      cflags |= PARODD;
    }

    cflags |= CFlags(m_baud_rate);


    if (m_flow_control == eFC_FLOW)
    {
      cflags |= CRTSCTS;
    }

    if (!m_use_modem_control)
    {
      cflags |= CLOCAL;
    }

    if (m_enable_receiver)
    {
      cflags |= CREAD;
    }

    if (m_enable_stop_on_receive)
    {
      cflags |= IXOFF;
    }

    return cflags;
  }

  unsigned long SerialFlags::CFlags(BaudRate baud_rate)
  {
    switch (baud_rate)
    {
      case SerialFlags::eBR_50:
        return B50;
      case SerialFlags::eBR_75:
        return B75;
      case SerialFlags::eBR_110:
        return B110;
      case SerialFlags::eBR_134:
        return B134;
      case SerialFlags::eBR_150:
        return B150;
      case SerialFlags::eBR_200:
        return B200;
      case SerialFlags::eBR_300:
        return B300;
      case SerialFlags::eBR_600:
        return B600;
      case SerialFlags::eBR_1200:
        return B1200;
      case SerialFlags::eBR_1800:
        return B1800;
      case SerialFlags::eBR_2400:
        return B2400;
      case SerialFlags::eBR_4800:
        return B4800;
      case SerialFlags::eBR_9600:
        return B9600;
      case SerialFlags::eBR_19200:
        return B19200;
      case SerialFlags::eBR_38400:
        return B38400;
      case SerialFlags::eBR_57600:
        return B57600;
      case SerialFlags::eBR_115200:
        return B115200;
      case SerialFlags::eBR_230400:
        return B230400;
      case SerialFlags::eBR_500000:
        return B500000;
      case SerialFlags::eBR_921600:
        return B921600;
      Default:
        return B0;
    }
  }

  #endif

  #ifdef _SYSTEM_WIN32_

  BYTE parity_map[] = {
    NOPARITY,
    EVENPARITY,
    ODDPARITY,
    MARKPARITY,
    SPACEPARITY
  };

  BYTE stopbit_map[] =  {
    ONESTOPBIT,
    ONE5STOPBITS,
    TWOSTOPBITS
  };

  void SerialFlags::GetDCB(LPDCB dcb) const
  {
    dcb->DCBlength = sizeof(DCB);
    dcb->fBinary = TRUE;
    dcb->fOutxCtsFlow = FALSE;
    dcb->fOutxDsrFlow = FALSE;
    dcb->fDtrControl = DTR_CONTROL_DISABLE;
    dcb->fDsrSensitivity = FALSE;
    dcb->fTXContinueOnXoff = FALSE;
    dcb->fOutX = FALSE;
    dcb->fInX = FALSE;
    dcb->fErrorChar = FALSE;
    dcb->fNull = FALSE;
    dcb->fRtsControl = RTS_CONTROL_DISABLE;
    dcb->fAbortOnError = FALSE;
    dcb->wReserved = 0U;

    dcb->ByteSize = m_data_bits;
    dcb->StopBits = stopbit_map[m_stop_bits];
    dcb->fParity = (m_parity != eP_NONE);
    dcb->Parity = parity_map[m_parity];
    dcb->BaudRate = m_baud_rate;

    //if (m_flow_control == eFC_FLOW)
    //{
    //  cflags |= CRTSCTS;
    //}
    //
    //if (!m_use_modem_control)
    //{
    //  cflags |= CLOCAL;
    //}
    //
    //if (m_enable_receiver)
    //{
    //  cflags |= CREAD;
    //}
    //
    //if (m_enable_stop_on_receive)
    //{
    //  dcb->fOutX = TRUE;
    //}
  }

  #endif

}
}
