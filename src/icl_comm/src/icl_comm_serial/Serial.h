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
 * \author Kay-Ulrich Scholl
 * \date 05.12.00
 * \author  Lars Pfotzer <pfotzer@fzi.de>
 * \date    2013-03-08
 *
 * \brief Contains a class that enables access to serial devices
 *
 * stole lots of the code from mcal_hardware/tSerial.*
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_COMM_SERIAL_SERIAL_H_INCLUDED
#define ICL_COMM_SERIAL_SERIAL_H_INCLUDED

#include <icl_core/BaseTypes.h>

#include "icl_comm_serial/ImportExport.h"
#include "icl_comm_serial/Logging.h"
#include "icl_comm_serial/SerialFlags.h"

#ifdef _SYSTEM_WIN32_
typedef unsigned int   speed_t;
typedef int ssize_t;
#endif

#ifdef _SYSTEM_POSIX_
# include <unistd.h>
# ifdef _SYSTEM_DARWIN_
#  include <termios.h>
# else
#  include <termio.h>
# endif
#endif

namespace icl_comm {
namespace serial {

//! Enables acces to serial devices
/*!
  Open a serial device, change baudrates, read from and write to the device.
  Status-information after calling the functions get be
 */
class ICL_COMM_SERIAL_IMPORT_EXPORT Serial
{
public:
  /*!
    Opens the device (if possible) with the given flags.
    \param flags can be values of the following values ORed together
    - CS5, CS6, CS7, or CS8  databits
    - CSTOPB set two stop bits, rather than one.
    - CREAD  enable receiver.
    - PARENB enable  parity  generation  on  output  and  parity
              checking for input.
    - PARODD parity for input and output is odd.
    - HUPCL  lower modem control lines after last process closes
              the device (hang up).
    - CLOCAL ignore modem control lines
    - B50, B75, B110, B134, B150, B200, B300, B600, B1200,
     B1800, B2400, B4800, B9600, B19200, B38400, B57600,
     B115200, B230400, B921600      Baudrates
    - CRTSCTS  flow control.
  */
  Serial(const char* dev_name, const SerialFlags& flags);

  /*!
   * This is an overloaded constructor, provided for convenience.
   * \a buad_rate overwrites the baud rate set in \a flags.
   */
  Serial(const char* dev_name, SerialFlags::BaudRate baud_rate, const SerialFlags& flags);

  /*!
    Restore old serial settings and close device
   */
  ~Serial();
  /*!
    speed is one of the followind values :
    - B50, B75, B110, B134, B150, B200, B300, B600, B1200,
    B1800, B2400, B4800, B9600, B19200, B38400, B57600,
    B115200, B230400, B921600
    Returns 0 on success.
    Returns -status on failure.
   */
  int ChangeBaudrate(SerialFlags::BaudRate speed);

  /*!
   * Clears the serial port's receive buffer.
   */
  int ClearReceiveBuffer();

  /*!
   * Clears the serial port's send buffer.
   */
  int ClearSendBuffer();

  /*!
    Opens the serial interface with the given \a flags
   */
  bool Open(const SerialFlags& flags)
  {
    m_serial_flags=flags;
    return Open();
  }

  /*!
    Opens the serial interface.
   */
  bool Open();

  /*!
    Returns \c true if the serial interface is opened.
    \c false otherwhise.
   */
  bool IsOpen() const;

  /*!
    Close the serial interface.
   */
  void Close();


  /*!
    Write data to serial out.
   */
  ssize_t Write(const void *data, ssize_t size);
  /*!
    Read data from device. This function waits until \param time us passed or
    the respected number of bytes are received via serial line.
    if (\param return_on_less_data is true (default value), the number of bytes
    that have been receives are returned and the data is stored in \param data.
    If the parameter is false, data is only read from serial line, if at least
    \param size bytes are available.
   */
  ssize_t Read(void *data, ssize_t size, unsigned long time = 100, bool return_on_less_data = true);
  /*!
    All routines return a negavtiv number on error. Then the global errno is
    stored into a private variable. Use this funtion to ask for this value.
    Especially the constructor cannot return an error-value. Check the
    succesful opening of the device by calling this function. It returns 0, if
    no error occured.
   */
  int Status() const
  {
    return m_status;
  }

  std::string StatusText() const;

  const char* DeviceName()const { return m_dev_name;}

  /*!
    Return the file descriptor of the serial class
  */
  int FileDescriptor()
  {
#ifdef _SYSTEM_POSIX_
    return file_descr;
#else
    return 0;
#endif
  }

private:
  // Forbid copying.
  Serial(const Serial&);
  Serial& operator = (const Serial&);

  void DumpData(void *data, size_t length);

#ifdef _SYSTEM_WIN32_
  HANDLE m_com;
  unsigned char m_read_buffer[0x4000];
  ssize_t m_read_buffer_fill;
#endif

#ifdef _SYSTEM_POSIX_
  int file_descr;
  termios io_set_old;

  bool is_lxrt_serial;
# ifdef _SYSTEM_LXRT_
  bool IsLXRTDeviceName(const char* device_name);
  int tty;
# endif
#endif

  char *m_dev_name;
  SerialFlags m_serial_flags;
  int m_status;

};

}
}

#endif
