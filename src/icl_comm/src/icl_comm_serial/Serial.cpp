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
 *
 * \author Kay-Ulrich Scholl
 * \date 05.12.00
 *
 *
 * \author  Lars Pfotzer <pfotzer@fzi.de>
 * \date    2013-03-08
 *
 *
 */
//----------------------------------------------------------------------

// Not functional with Mac OS X (UH)

#include "icl_comm_serial/Serial.h"

#include <cassert>
#include <algorithm>

#include <icl_core/os_lxrt.h>

#ifdef _SYSTEM_LXRT_
# include <sys/mman.h>
# include <rtai_serial.h>
#endif

#ifdef _SYSTEM_LINUX_
# include <stdio.h>
# include <fcntl.h>
# include <sys/time.h>
# include <errno.h>
# include <string.h>
#endif

#include "icl_core/TimeStamp.h"
#include "icl_core/TimeSpan.h"
//#include "mcal_math/OwnMath.h"

#ifdef _SYSTEM_WIN32_
#include <math.h>
#include <stdio.h>
#endif

#undef SERIAL_DUMP_DATA
//#define SERIAL_DUMP_DATA

// Work around a typo in older RTAI serial drivers.
#if defined(_SYSTEM_LXRT_33_) || defined(_SYSTEM_LXRT_35_)
# define rt_spset_mcr(tty, mask, setbits) rt_spset_msr(tty, mask, setbits)
#endif

namespace icl_comm  {
namespace serial {

  Serial::Serial(const char *dev_name, const SerialFlags& flags)
    :m_dev_name(strdup(dev_name)),
     m_serial_flags(flags)
  {
  #ifdef _SYSTEM_WIN32_
    m_com=INVALID_HANDLE_VALUE;
  #else
    file_descr=-1;
    is_lxrt_serial = false;
  #endif

    Open();
  }

  Serial::Serial(const char *dev_name, SerialFlags::BaudRate baud_rate, const SerialFlags& flags)
    :m_dev_name(strdup(dev_name)),
     m_serial_flags(flags)
  {
  #ifdef _SYSTEM_WIN32_
    m_com=INVALID_HANDLE_VALUE;
  #else
    file_descr=-1;
    is_lxrt_serial = false;
  #endif

    m_serial_flags.setBaudRate(baud_rate);
    Open();
  }

  bool Serial::Open()
  {
    Close();

  #if defined _SYSTEM_LINUX_
  #ifdef _SYSTEM_LXRT_
    if (icl_core::os::IsThisLxrtTask() && IsLXRTDeviceName(m_dev_name))
    {
      is_lxrt_serial = true;
      int baud= m_serial_flags.getBaudRate();
      int numbits;
      switch (m_serial_flags.DataBits())
      {
        case SerialFlags::eDB_5:
          numbits = 5;
          break;
        case SerialFlags::eDB_6:
          numbits = 6;
          break;
        case SerialFlags::eDB_7:
          numbits = 7;
          break;
        case SerialFlags::eDB_8:
          numbits = 8;
          break;
        default:
          numbits = 8;
      }

      int stopbits;
      if (m_serial_flags.StopBits())
        stopbits = 2;
      else
        stopbits = 1;

      int parity=0;
      switch (m_serial_flags.Parity())
      {
        case SerialFlags::eP_EVEN:
        {
          parity = RT_SP_PARITY_EVEN;
          break;
        }
        case SerialFlags::eP_ODD:
        {
          parity = RT_SP_PARITY_ODD;
          break;
        }
        case SerialFlags::eP_NONE:
        case SerialFlags::eP_MARK:
        case SerialFlags::eP_SPACE:
        {
          parity = RT_SP_PARITY_NONE;
          break;
        }
      }

      int mode=0;
      switch (m_serial_flags.FlowControl())
      {
        case SerialFlags::eFC_FLOW:
        {
          mode = RT_SP_HW_FLOW;
          break;
        }
        case SerialFlags::eFC_HAND_SHAKE:
        {
          mode = RT_SP_NO_HAND_SHAKE;
          break;
        }
      }

      int fifotrig = RT_SP_FIFO_SIZE_DEFAULT;

      LOGGING_INFO_C(SERIAL, Serial, "Using LXRT extension for accessing serial device " << tty << " (" << m_dev_name << "): " <<
                      baud << " " << numbits << " " << stopbits << " " << parity << " " << mode << " " << fifotrig << endl);
      //INFOMSG("Using LXRT extension for accessing serial device %i (%s): %i %i %i %x %x %i\n", tty, m_dev_name, baud, numbits, stopbits, parity, mode, fifotrig);
      if ((m_status = rt_spopen(tty, baud, numbits, stopbits, parity, mode, fifotrig)) < 0)
      {
        LOGGING_DEBUG_C(SERIAL, Serial, "Cannot open lxrt serial device '" << tty << " (" << m_dev_name << ")'. Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Cannot open lxrt serial device '%i (%s)'. Status (%i:%s)\n", tty, m_dev_name, m_status, strerror(-m_status));
        return false;
      }
      else
      {
        file_descr = tty;
        m_status = 0;

        if (m_serial_flags.getModemControlFlags()!=SerialFlags::eMCF_UNDEFINED)
        {
          LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ") setting hardware modem control flags to 0x" << m_serial_flags.getModemControlFlags() << endl);
          //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s) setting hardware modem control flags to 0x%x\n", m_dev_name, m_serial_flags.getModemControlFlags());

          int mask = 0;
          // setting the mask to all set bits
          if (m_serial_flags.getModemControlFlags() & SerialFlags::eMCF_RTS)
            mask |= RT_SP_RTS;
          if (m_serial_flags.getModemControlFlags() & SerialFlags::eMCF_DTR)
            mask |= RT_SP_DTR;

          int ret = rt_spset_mcr(tty, mask, 1);
          //LDM("rt_spset_mcr(%i, %x, 1)=%i\n", tty, mask, ret);

          // the zero-bits also have to be cleared explicitly, so change the mask for that
          // setting the mask to all bits not set
          mask = 0;
          if (~(m_serial_flags.getModemControlFlags() & SerialFlags::eMCF_RTS))
            mask |= RT_SP_RTS;
          if (~(m_serial_flags.getModemControlFlags() & SerialFlags::eMCF_DTR))
            mask |= RT_SP_DTR;

          ret = rt_spset_mcr(tty, mask, 0);
          //LDM("rt_spset_mcr(%i, %x, 0)=%i\n", tty, mask, ret);
        }
      }
    }
    else
  #endif
    // Attention! The following code will be executed,
    // if we are not a lxrt-task or no lxrt interface is available:
    {

      termios io_set_new;

      // open device
      if ((file_descr = open(m_dev_name, O_RDWR | O_NONBLOCK)) < 0)
      {
        m_status = -errno;
        LOGGING_DEBUG_C(SERIAL, Serial, "Cannot open serial device '" << m_dev_name << "'. Status (" << m_status << ":" << strerror(-m_status) << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Cannot open serial device '%s'. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
        return false;
      }
      else
        m_status = 0;

      // get device-settings
      if (tcgetattr(file_descr, &io_set_old) < 0)
      {
        m_status = -errno;
        LOGGING_DEBUG_C(SERIAL, Serial,  "Cannot get serial device m_status '" << m_dev_name << "'. Status (" << m_status << ":" << strerror(-m_status) << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Cannot get serial device m_status '%s'. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
        return false;
      }
      else
        m_status = 0;

      // copy settings from old settings
      io_set_new = io_set_old;

      // declare new settings
      io_set_new.c_cflag = m_serial_flags.CFlags();
      io_set_new.c_oflag = 0;
      io_set_new.c_iflag = IGNPAR;
      io_set_new.c_lflag = 0;
      io_set_new.c_cc[VMIN] = 1;
      io_set_new.c_cc[VTIME] = 0;

      // set new settings
      if (tcsetattr(file_descr, TCSANOW, &io_set_new) < 0)
      {
        m_status = -errno;
        LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ") Error>> tcsetattr failed. Status (" << m_status << ":" << strerror(-m_status) << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s) Error>> tcsetattr failed. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
        return false;
      }
      else
        m_status = 0;

      if (m_serial_flags.getModemControlFlags()!=SerialFlags::eMCF_UNDEFINED)
      {
        LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ") setting hardware modem control flags to 0x" << m_serial_flags.getModemControlFlags() << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s) setting hardware modem control flags to 0x%x\n", m_dev_name, m_serial_flags.getModemControlFlags());
        int modem_control_flags=0;
        if (m_serial_flags.getModemControlFlags() & SerialFlags::eMCF_DTR)
        {
          modem_control_flags |= TIOCM_DTR;
        }
        if (m_serial_flags.getModemControlFlags() & SerialFlags::eMCF_RTS)
        {
          modem_control_flags |= TIOCM_RTS;
        }

        ioctl(file_descr, TIOCMSET, modem_control_flags);
      }
    }

  #elif defined _SYSTEM_WIN32_

    m_read_buffer_fill = 0;
    m_status = 0;

    // Try to open the serial port.
    m_com = CreateFile(m_dev_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (m_com == INVALID_HANDLE_VALUE)
    {
      m_status = GetLastError();
      LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> open port failed (" << StatusText().c_str() << ")." << endl);
      //WARNINGMSG("Serial(%s): ERROR>> open port failed (%s).\n", m_dev_name, StatusText().c_str());
    }

    // Set the receive and send buffer size.
    if (m_status == 0)
    {
      if (!SetupComm(m_com, 0x4000, 0x4000))
      {
        m_status = GetLastError();
        LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> SetupComm failed (" << StatusText().c_str() << ")." << endl);
        //WARNINGMSG("Serial(%s): ERROR>> SetupComm failed (%s).\n", m_dev_name, StatusText().c_str());
      }
    }

    // Get the current serial port configuration.
    DCB dcb;
    memset(&dcb, 0, sizeof(DCB));
    if (m_status == 0)
    {
      if (!GetCommState(m_com, &dcb))
      {
        m_status = GetLastError();
        LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> GetCommState failed (" << StatusText().c_str() << ")." << endl);
        //WARNINGMSG("Serial(%s): ERROR>> GetCommState failed (%s).\n", m_dev_name, StatusText().c_str());
      }
    }

    // Set the serial port configuration from the supplied seiral flags.
    if (m_status == 0)
    {
      m_serial_flags.GetDCB(&dcb);
      if (!SetCommState(m_com, &dcb))
      {
        m_status = GetLastError();
        LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> SetCommState failed (" << StatusText().c_str() << ")." << endl);
        //WARNINGMSG("Serial(%s): ERROR>> SetCommState failed (%s).\n", m_dev_name, StatusText().c_str());
        return false;
      }
    }
  #endif

    return m_status==0;
  }

  void Serial::DumpData(void *data, size_t length)
  {
    unsigned char *c_data = static_cast<unsigned char *>(data);
    printf("Serial::DumpData: ");
    for (size_t i = 0; i < length; ++i)
    {
      printf("%02X ", int(c_data[i]));
    }
    printf("\n");
  }

  int Serial::ChangeBaudrate(SerialFlags::BaudRate speed)
  {
    // Nothing to be done here.
    if (m_serial_flags.getBaudRate()==speed)
      {
        // success
        return 0;
      }
    m_serial_flags.setBaudRate(speed);

  #if defined _SYSTEM_LINUX_

  # ifdef _SYSTEM_LXRT_
    if (is_lxrt_serial)
    {
      if (icl_core::os::IsThisLxrtTask())
      {
        // don't perform open check in lxrt, because here we open/close the device in case a different baudrate is selected!
        Close();
        Open();
      }
      else
      {
        LOGGING_DEBUG_CO(SERIAL, Serial, ChangeBaudrate, "Serial Error>> could not change baudrate of a lxrt serial in non lxrt thread." << endl);
      }
    }
    else
  # endif
    {
      if (file_descr < 0)
        return m_status;

      struct termios io_set;

      // Get device settings
      if (tcgetattr(file_descr, &io_set) < 0)
      {
        m_status = -errno;
        LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << "): Error>> tcgetattr failed. Status (" << m_status << ":" << strerror(-m_status) << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial Error>> tcgetattr (%s) failed. m_status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
      }
      else
      {
        // clear speed-settings
        io_set.c_cflag &= ~CBAUD;
        // add new speed-settings
        io_set.c_cflag |= SerialFlags::CFlags(speed);

        // set new device settings
        if (tcsetattr(file_descr, TCSANOW, &io_set) < 0)
        {
          m_status = -errno;
          LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << "): Error>> tcsetattr failed. Status (" << m_status << ":" << strerror(-m_status) << endl);
          //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s) Error>> tcsetattr failed. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
        }
        else
        {
          LOGGING_INFO_C(SERIAL, Serial, "Serial:ChangeBaudrate " << speed << " successful." << endl);
          //INFOMSG("Serial:ChangeBaudrate %i successful\n", speed);
          m_status = 0;
        }
      }
    }

    return m_status;
  #endif

  #ifdef _SYSTEM_DARWIN_
    LOGGING_WARNING_C(SERIAL, Serial, "Serial:changeBaudrate() >> to be implemented" << endl);
    //WARNINGMSG("Serial:changeBaudrate() >> to be implemented\n");
    return -1;
  #endif

  #ifdef _SYSTEM_WIN32_
    m_status = 0;

    // Clears the output and input buffers.
    if (!PurgeComm(m_com, PURGE_RXCLEAR)
        || !PurgeComm(m_com, PURGE_TXCLEAR))
    {
      m_status = GetLastError();
      LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> PurgeComm failed (" << StatusText().c_str() << ")." << endl);
      //WARNINGMSG("Serial(%s): ERROR>> PurgeComm failed (%s).\n", m_dev_name, StatusText().c_str());
    }

    // Get the current serial port configuration.
    DCB dcb;
    memset(&dcb, 0, sizeof(DCB));
    if (m_status == 0)
    {
      if (!GetCommState(m_com, &dcb))
      {
        m_status = GetLastError();
        LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> GetCommState failed (" << StatusText().c_str() << ")." << endl);
        //WARNINGMSG("Serial(%s): ERROR>> GetCommState failed (%s).\n", m_dev_name, StatusText().c_str());
      }
    }

    // Set the serial port configuration with the new baud rate.
    if (m_status == 0)
    {
      m_serial_flags.GetDCB(&dcb);
      if (!SetCommState(m_com, &dcb))
      {
        m_status = GetLastError();
        LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> SetCommState failed (" << StatusText().c_str() << ")." << endl);
        //WARNINGMSG("Serial(%s): ERROR>> SetCommState failed (%s).\n", m_dev_name, StatusText().c_str());
      }
    }

    return -m_status;
  #endif
  }

  int Serial::ClearReceiveBuffer()
  {
  #ifdef _SYSTEM_WIN32_
    m_status = 0;

    // Clears the input buffers.
    if (PurgeComm(m_com, PURGE_RXCLEAR))
    {
      m_read_buffer_fill = 0;
    }
    else
    {
      m_status = GetLastError();
      LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> PurgeComm failed (" << StatusText().c_str() << ")." << endl);
      //WARNINGMSG("Serial(%s): ERROR>> PurgeComm failed (%s).\n", m_dev_name, StatusText().c_str());
    }

    return m_status;
  #elif defined _SYSTEM_LINUX_
    // could not test for LXRT device, so return -1 to be on the safe side
  # ifdef _SYSTEM_LXRT_
    return -1;
  #endif
    if (tcflush(file_descr, TCIFLUSH) != 0)
    {
      LOGGING_WARNING_C(SERIAL, Serial, "tcflush failed :(" << endl);
      return -1;
    }
  #else
    return -1;
  #endif
    return 0;
  }

  int Serial::ClearSendBuffer()
  {
  #ifdef _SYSTEM_WIN32_
    m_status = 0;

    // Clears the output buffers.
    if (PurgeComm(m_com, PURGE_TXCLEAR))
    {
      m_read_buffer_fill = 0;
    }
    else
    {
      m_status = GetLastError();
      LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> PurgeComm failed (" << StatusText().c_str() << ")." << endl);
      //WARNINGMSG("Serial(%s): ERROR>> PurgeComm failed (%s).\n", m_dev_name, StatusText().c_str());
    }

    return m_status;
  #elif defined _SYSTEM_LINUX_
    // could not test for LXRT device, so return -1 to be on the safe side
  # ifdef _SYSTEM_LXRT_
    return -1;
  #endif
    if (tcflush(file_descr, TCOFLUSH) != 0)
    {
      LOGGING_WARNING_C(SERIAL, Serial, "tcflush failed :(" << endl);
      return -1;
    }
  #else
    return -1;
  #endif
    return 0;
  }

  ssize_t Serial::Write(const void *data, ssize_t size)
  {
  #if defined _SYSTEM_LINUX_
    if (file_descr < 0)
      return m_status;

    int bytes_out = 0;

  # ifdef _SYSTEM_LXRT_
    if (is_lxrt_serial)
    {
      if (icl_core::os::IsThisLxrtTask())
      {
        int bytes_to_send=size;
        int bytes_sent=0;
        while (bytes_to_send>0)
        {
          int bytes_not_sent=rt_spwrite(tty, ((char*)data)+bytes_sent, bytes_to_send);
          if (bytes_not_sent < 0)
          {
            m_status = bytes_not_sent;
            LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ":" << tty << "): Error on lxrt-writing. Status (" << m_status << ":" << strerror(-m_status) << ")." << endl);
            //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s:%i) Error on lxrt-writing. Status (%i:%s)\n", m_dev_name, tty, m_status, strerror(-m_status));
            //LDM("rt_write(%i, data, %i) = failed (%i)\n", tty, bytes_to_send, bytes_not_sent);
            // stop it
            bytes_to_send = 0;
          }
          else
          {
            m_status = 0;
            bytes_sent += bytes_to_send - bytes_not_sent;
            //LDM("rt_write(%i, data, %i) = %i\n", tty, bytes_to_send, bytes_not_sent);
            // these bytes are left
            bytes_to_send = bytes_not_sent;
          }
        }
      }
      else
      {
        LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ":" << tty << "): Error>> could not write to a lxrt serial in non lxrt thread." << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s:%i) Error>> could not write to a lxrt serial in non lxrt thread.\n", m_dev_name, tty);
      }
    }
    else
  # endif
    {
      // just write it to device
      if ((bytes_out = write(file_descr, (char*)data, size)) < 0)
      {
        m_status = -errno;
        LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ":" << m_dev_name << "): Error on writing. Status (" << m_status << ":" << strerror(-m_status) << ")." << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s) Error on writing. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
      }
      else
      {
        m_status = 0;
      }
    }

    return bytes_out;

  #elif defined _SYSTEM_WIN32_

    assert(m_com != INVALID_HANDLE_VALUE);

    DWORD bytes_written;
    if (!WriteFile(m_com, data, size, &bytes_written, NULL))
    {
      m_status = GetLastError();
      LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> could not write data (" << StatusText().c_str() << ")." << endl);
      //WARNINGMSG("Serial(%s): ERROR>> could not write data (%s).\n", m_dev_name, StatusText().c_str());
    }
    else
    {
      m_status = 0;
  #ifdef SERIAL_DUMP_DATA
      DumpData(data, bytes_written);
  #endif
    }

    return (m_status == 0 ? bytes_written : -m_status);
  #else
    return -1;
  #endif
  }

  ssize_t Serial::Read(void *data, ssize_t size, unsigned long time, bool return_on_less_data)
  {
    //tTime end_time = tTime().FutureUSec(time);
    icl_core::TimeStamp end_time = icl_core::TimeStamp::futureMSec(time / 1000);

  #if defined _SYSTEM_LINUX_
    if (file_descr < 0)
      return m_status;

    icl_core::TimeSpan tz;
    fd_set fds;
    int bytes_read  = 0;
    int bytes_read_inc;
    int select_return;
    char *buffer = (char*)data;

    if (time <= 0) time = 1;
    //LDM("Serial(%s)::Read(%i) (time left %li us)\n", m_dev_name, size, time);

    m_status = 0;
  # ifdef _SYSTEM_LXRT_
    if (is_lxrt_serial)
    {
      if (icl_core::os::IsThisLxrtTask())
      {
        int msg_size = size;
        RTIME timeout_relative = time;
        timeout_relative = nano2count(timeout_relative * 1000);
        int read_return;
        // not received the whole message, so if configured for reading on less data: read what you can get
        if ((read_return = rt_spread_timed(tty, (char*)data, msg_size, timeout_relative)) != 0)
        {
          //LDM("rt_spread_timed(%i, data, %i) = %i (time left %li us) => bytes read = 0\n", tty, msg_size, read_return, (end_time - icl_core::TimeStamp::now()).tsUSec());
          if (return_on_less_data)
          {
            read_return = rt_spread(tty, (char*)data, msg_size);

            if (read_return == 0)
            {
              LOGGING_ERROR_CO(SERIAL, Serial, Read, "Serial(" << m_dev_name << ")::Read(" << size << ": rt_spread_timed should have returned with data." << endl);
              //ERRORMSG("Serial(%s)::Read(%i): rt_spread_timed should have returned with data\n", m_dev_name, size);
            }

            if (read_return < 0)
            {
              m_status = read_return;
              LOGGING_DEBUG_CO(SERIAL, Serial, Read, "Error on reading " << tty " (" << m_dev_name << "). Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
              //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Error on reading %i (%s). Status (%i:%s)\n", tty, m_dev_name, m_status, strerror(-m_status));

              icl_core::TimeSpan excess_read_time = icl_core::TimeStamp::now() - end_time;
              if (excess_read_time > icl_core::TimeSpan().fromMSec(1))
              {
                LOGGING_ERROR_CO(SERIAL, Serial, Read, "Serial(" << m_dev_name << ")::Read(" << size << "took too long (" << excess_read_time.toUSec() << "us)" << endl);
                //ERRORMSG("Serial(%s)::Read(%i) took too long (%ius)\n", m_dev_name, size, excess_read_time.ToUSec());
              }

              return m_status;
            }
            bytes_read = size - read_return;
            //LDM("rt_spread(%i, data, %i) = %i (time left %li us), bytes read = %i\n", tty, msg_size, read_return, (end_time - icl_core::TimeStamp::now()).tsUSec(), bytes_read);
          }
        }
        else
        {
          bytes_read = msg_size;
          //LDM("rt_spread_timed(%i, data, %i) = 0 (time left %li us) => bytes read = %i\n", tty, msg_size, (end_time - icl_core::TimeStamp::now()).tsUSec(), bytes_read);
        }

        icl_core::TimeSpan() excess_read_time = icl_core::TimeStamp::now() - end_time;
        if (excess_read_time > icl_core::TimeSpan().fromMSec(1))
        {
          LOGGING_ERROR_CO(SERIAL, Serial, Read, "Serial(" << m_dev_name << ")::Read(" << size << "took too long (" << excess_read_time.toUSec() << "us)" << endl);
          //ERRORMSG("Serial(%s)::Read(%i) took too long (%ius)\n", m_dev_name, size, excess_read_time.ToUSec());
        }
      }
      else
      {
        LOGGING_DEBUG_CO(SERIAL, Serial, "Serial(" << m_dev_name << ":" << tty << "). Error>> could not read from a lxrt serial in non lxrt thread." << endl);
        //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial(%s:%i) Error>> could not read from a lxrt serial in non lxrt thread.\n", m_dev_name, tty);
      }
    }
    else
  # endif
    {
      // We wait max time
      do
      {
        tz = end_time - icl_core::TimeStamp::now();
        // min 1 us, otherwise we will read nothing at all
        if (tz < icl_core::TimeSpan(0, 1000))
        {
          tz = icl_core::TimeSpan(0, 1000);
        }

        //LDM("Serial(%s) Check Read.\n", m_dev_name);

        FD_ZERO(&fds);
        FD_SET(file_descr, &fds);
        // Look for received data:
        if ((select_return = select(FD_SETSIZE, &fds, 0, 0, (timeval*) & tz)) > 0)
        {
          //LDM("Serial(%s) Select successful.\n", m_dev_name);
          if (return_on_less_data)
          {
            if ((bytes_read_inc = read(file_descr, &buffer[bytes_read], size - bytes_read)) < 0)
            {
              m_status = -errno;
              LOGGING_DEBUG_C(SERIAL, Serial, "Error on reading '" << m_dev_name << "'. Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
              //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Error on reading '%s'. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
              return m_status;
            }
            // Any bytes read ?
            if (bytes_read_inc > 0)
            {
              bytes_read += bytes_read_inc;
              //LDM("Serial(%s) Data read (%i => %i).\n", m_dev_name, bytes_read_inc, bytes_read);
              if (bytes_read == size)
              {
                return bytes_read;
              }
            }
          }
          else
          {
            //LDM("serial:time left %lu\n",Time2Long(tz));
            // Are there already enough bytes received ?
            if (ioctl(file_descr, FIONREAD, &bytes_read_inc) < 0)
            {
              m_status = -errno;
              LOGGING_DEBUG_C(SERIAL, Serial, "Error on ioctl FIONREAD '" << m_dev_name << "'. Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
              //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Error on ioctl FIONREAD '%s'. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
              return m_status;
            }
            else
            {
              // Yes? then read data
              if (bytes_read_inc >= size)
              {
                if ((bytes_read = read(file_descr, buffer, size)) < 0)
                {
                  m_status = -errno;
                  LOGGING_DEBUG_C(SERIAL, Serial, "Error on read '" << m_dev_name << "'. Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
                  //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Error on read '%s'. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
                  return m_status;
                }
                else
                {
                  //LDM("Serial(%s) Data read (%i bytes).\n", m_dev_name, bytes_read);
                  return bytes_read;
                }
              }
              // No ? do nothing
            }
          }
        }
        else if (select_return < 0)
        {
          m_status = -errno;
          LOGGING_DEBUG_C(SERIAL, Serial, "Error on select '" << m_dev_name << "'. Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
          //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Error on select '%s'. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
          return m_status;
        }
      }
      // Look again for data, if any time left
      while (icl_core::TimeStamp::now() < end_time);
    }

    //LDM("Serial(%s)::Read(%i) (time left %li us) = %i.\n", m_dev_name, size, (end_time - icl_core::TimeStamp::now()).toUSec(), bytes_read);
    return bytes_read;
  #endif


  #ifdef _SYSTEM_DARWIN_
    LOGGING_WARNING_C(SERIAL, Serial, "Serial:Read() >> to be implemented!" << endl);
    //WARNINGMSG("Serial:Read() >> to be implemented\n");
    return 0;
  #endif

  #ifdef _SYSTEM_WIN32_
    assert(m_com != INVALID_HANDLE_VALUE);

    m_status = 0;

    // Set the timeout for the read operation.
    COMMTIMEOUTS timeout;
    timeout.ReadIntervalTimeout = time / 1000;
    timeout.ReadTotalTimeoutMultiplier = 1;
    timeout.ReadTotalTimeoutConstant = time / 1000;
    timeout.WriteTotalTimeoutMultiplier = 1;
    timeout.WriteTotalTimeoutConstant = MAXDWORD;
    if (!SetCommTimeouts(m_com, &timeout))
    {
      m_status = GetLastError();
      LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> setting read timeout failed (" << StatusText().c_str() << ")" << endl);
      //WARNINGMSG("Serial(%s): ERROR>> setting read timeout failed (%s).\n", m_dev_name, StatusText().c_str());
    }

    // Try to receive data.
    if (m_read_buffer_fill <= size
        && m_status == 0)
    {
      DWORD bytes_received = 0;
      size_t bytes_remaining = (m_read_buffer_fill < size ? size - m_read_buffer_fill : 0);
      icl_core::TimeStamp now;
      do
      {
        if (ReadFile(m_com, m_read_buffer + m_read_buffer_fill, bytes_remaining, &bytes_received, NULL))
        {
          m_read_buffer_fill += bytes_received;
          if (bytes_remaining > bytes_received)
          {
            bytes_remaining -= bytes_received;
          }
          else
          {
            bytes_remaining = 0;
          }
        }
        else
        {
          DWORD error = GetLastError();
          if (error != ERROR_TIMEOUT)
          {
            m_status = error;
            LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> error during read (" << StatusText().c_str() << ")" << endl);
            //WARNINGMSG("Serial(%s): ERROR>> error during read (%s).\n", m_dev_name, StatusText().c_str());
          }
        }
        now = icl_core::TimeStamp::now();
      }
      while (m_status == 0 && !return_on_less_data && bytes_remaining && now < end_time);

      if (m_status == 0 && !return_on_less_data && bytes_remaining && now >= end_time)
      {
        m_status = ERROR_TIMEOUT;
        LOGGING_WARNING_C(SERIAL, Serial, "Serial(" << m_dev_name << "): ERROR>> error during read (" << StatusText().c_str() << ")" << endl);
        //WARNINGMSG("Serial(%s): ERROR>> error during read (%s).\n", m_dev_name, StatusText().c_str());
      }
    }

    // Copy data to output buffer.
    ssize_t bytes_received;
    if (m_status == 0)
    {
      bytes_received = std::min(size, m_read_buffer_fill);
      memcpy(data, m_read_buffer, bytes_received);

      // \todo Make this more efficient.
      for (ssize_t write_index = 0, read_index = bytes_received; read_index < m_read_buffer_fill; ++write_index, ++read_index)
      {
        m_read_buffer[write_index] = m_read_buffer[read_index];
      }
      m_read_buffer_fill -= bytes_received;

  #ifdef SERIAL_DUMP_DATA
      DumpData(data, bytes_received);
  #endif
    }

    return (m_status == 0 ? bytes_received : -m_status);
  #endif
  }

  std::string Serial::StatusText() const
  {
  #if defined _SYSTEM_LINUX_
    return strerror(-m_status);
  #elif defined _SYSTEM_WIN32_
    LPVOID msg_buff = NULL;
    DWORD res = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                              NULL,
                              m_status, //  from GetLastError(),
                              MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
                              (LPTSTR) &msg_buff,
                              0,
                              NULL);
    std::string result;
    if (res)
    {
      result = (LPCTSTR)msg_buff;
      LocalFree(msg_buff);
    }
    else
    {
      result = "fatal internal error";
    }
    return result;
  #else
    return "Plattform not supported!";
  #endif
  }

  bool Serial::IsOpen() const
  {
  #ifdef _SYSTEM_LINUX_
    return file_descr >= 0;
  #elif defined _SYSTEM_WIN32_
    return m_com != INVALID_HANDLE_VALUE;
  #else
    return false;
  #endif
  }

  void Serial::Close()
  {
    //LDM("Serial::Close\n");
  #ifdef _SYSTEM_LINUX_
    if (file_descr >= 0)
    {
  # ifdef _SYSTEM_LXRT_
      if (is_lxrt_serial)
      {
        if (icl_core::os::IsThisLxrtTask())
        {
          //LDM("Serial lxrt close device %i\n", tty);
          if ((m_status = rt_spclose(tty)) < 0)
          {
            LOGGING_DEBUG_C(SERIAL, Serial, "Serial Error>> closing lxrt serial " << tty << " (" << m_dev_name << "). Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
            //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial Error>> closing lxrt serial %i (%s). Status(%i:%s)\n", tty, m_dev_name, m_status, strerror(-m_status));
          }
        }
        else
        {
            LOGGING_DEBUG_C(SERIAL, Serial, "Serial Error>> could not close a lxrt serial in non lxrt thread." << endl);
          //DEBUGMSG(DD_SYSTEM, DL_CREATE, "Serial Error>> could not close a lxrt serial in non lxrt thread.\n");
        }
      }
      else
  # endif
      {
        // restore old setting
        if (tcsetattr(file_descr, TCSANOW, &io_set_old) < 0)
        {
          m_status = -errno;
          LOGGING_DEBUG_C(SERIAL, Serial, "Serial(" << m_dev_name << ") Error>> tcsetattr failed. Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
          //DEBUGMSG(DD_SYSTEM, DL_CREATE, "Serial(%s) Error>> tcsetattr failed. Status (%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
        }
        // close device
        if (close(file_descr) < 0)
        {
          m_status = -errno;
          LOGGING_DEBUG_C(SERIAL, Serial, "Serial>> Error closing serial " << m_dev_name << ". Status (" << m_status << ":" << strerror(-m_status) << ")" << endl);
          //DEBUGMSG(DD_SYSTEM, DL_CREATE, "Serial>> Error closing serial %s. Status(%i:%s)\n", m_dev_name, m_status, strerror(-m_status));
        }
      }

      file_descr=-1;
      is_lxrt_serial = false;
    }
  #endif

  #ifdef _SYSTEM_WIN32_
    if (m_com != INVALID_HANDLE_VALUE)
    {
      CloseHandle(m_com);    // close device
      m_com=INVALID_HANDLE_VALUE;
    }
  #endif
    m_status=0;
    //LDM("Serial::Close done.\n");
  }

  Serial::~Serial()
  {
    //LDM("~Serial start\n");

    Close();
//    free(m_dev_name);
    m_dev_name = NULL;

    //LDM("~Serial done\n");
  }


  #ifdef _SYSTEM_LXRT_
  bool Serial::IsLXRTDeviceName(const char* device_name)
  {
    char *check_pointer;
    tty = strtol(device_name, &check_pointer, 10);
    if (*check_pointer == 0)
    {
      LOGGING_DEBUG_C(SERIAL, Serial,"Serial::IsLXRTDeviceName(" << m_dev_name << ") Using LXRT-serial-device " << tty << endl);
      //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial::IsLXRTDeviceName(%s) Using LXRT-serial-device %i\n", m_dev_name, tty);
      return true;
    }
    else
    {
      LOGGING_DEBUG_C(SERIAL, Serial, "Serial::IsLXRTDeviceName(" << m_dev_name << ") Using LINUX-serial-device " << tty << " (non lxrt access)" endl);
      //DEBUGMSG(DD_SYSTEM, DL_DEBUG, "Serial::IsLXRTDeviceName(%s) Using LINUX-serial-device %s (non lxrt access)\n", m_dev_name, m_dev_name);
      return false;
    }
  }
  #endif

}
}
