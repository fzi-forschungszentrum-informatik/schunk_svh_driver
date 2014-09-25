// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \date    2014-02-03
 * \date    2014-07-16
 *
 * This file contains the ReceiveThread for the serial communication.
 * In order to receive packages independently from the sending direction
 * this thread periodically polls the serial interface for new data. If data
 * is present a statemachine will evaluate the right packet structure and send the
 * data to further parsing once a complete serial packaged is received
 */
//----------------------------------------------------------------------
#include <driver_svh/SVHReceiveThread.h>
#include <driver_svh/Logging.h>


using icl_comm::ArrayBuilder;

namespace driver_svh {

SVHReceiveThread::SVHReceiveThread(const TimeSpan& period, boost::shared_ptr<Serial> device,
                                     ReceivedPacketCallback const & received_callback)
  : PeriodicThread("SVHReceiveThread", period),
    m_serial_device(device),
    m_received_state(eRS_HEADER1),
    m_length(0),
    m_data(0, 0),
    m_ab(0),
    m_packets_received(0),
    m_received_callback(received_callback)
{}

void SVHReceiveThread::run()
{
  while (execute())
  {
    if (m_serial_device)// != NULL)
    {
      if (m_serial_device->IsOpen())
      {
        // All we every want to do is receiving data :)
        receiveData();
      }
      else
      {
        LOGGING_WARNING_C(DriverSVH, SVHReceiveThread, "Cannot read data from serial device. It is not opened!" << endl);
      }
    }

    // Wait for the thread period so that the timing is in sync.
    waitPeriod();
  }
}

bool SVHReceiveThread::receiveData()
{

  /*
   * Each packet has to follow the defined packet structure which is ensured by the following state machine.
   * The "Bytestream" (not realy a stream) is interpreted byte by byte. If the structure is still right the
   * next state is entered, if a wrong byte is detected the whole packet is discarded and the SM switches to
   * the synchronization state aggain.
   * If the SM reaches the final state the packet will be given to the packet handler to decide what to do with
   * its content.
   *  NOTE: All layers working with a SerialPacket (except this one) assume that the packet has a valid structure
   *        and all data fields present.
   */

  switch (m_received_state)
  {
    case eRS_HEADER1:
    {
      uint8_t data_byte = 0;
      if (m_serial_device->Read(&data_byte, sizeof(uint8_t)))
      {
        if (data_byte == PACKET_HEADER1)
        {
          m_received_state = eRS_HEADER2;
        }
      }
      break;
    }
    case eRS_HEADER2:
    {
      uint8_t data_byte = 0;
      if (m_serial_device->Read(&data_byte, sizeof(uint8_t)))
      {
        switch (data_byte)
        {
          case PACKET_HEADER2:
          {
            m_received_state = eRS_INDEX;
            break;
          }
          case PACKET_HEADER1:
          {
            m_received_state = eRS_HEADER2;
            break;
          }
          default:
          {
            m_received_state = eRS_HEADER1;
            break;
          }
        }
      }
      break;
    }
    case eRS_INDEX:
    {
      // Reset Array Builder for each fresh packet
      m_ab.reset(0);

      uint8_t index = 0;
      if (m_serial_device->Read(&index, sizeof(uint8_t)))
      {
        // Data bytes are not cenverted in endianess at this point
        m_ab.appendWithoutConversion(index);
        m_received_state = eRS_ADDRESS;
      }
      break;
    }
    case eRS_ADDRESS:
    {
      //get the address
      uint8_t address = 0;
      if (m_serial_device->Read(&address, sizeof(uint8_t)))
      {
        m_ab.appendWithoutConversion(address);
        m_received_state = eRS_LENGTH;
      }
      break;
    }
    case eRS_LENGTH:
    {
      // get payload length
      uint16_t length = 0;
      if (m_serial_device->Read(&length, sizeof(uint16_t)))
      {
        m_ab.appendWithoutConversion(length);
        m_length = m_ab.readBack<uint16_t>();
        m_received_state = eRS_DATA;
      }
      break;
    }
    case eRS_DATA:
    {
      // get the payload itself
      // Some conversion due to legacy hardware calls
      uint8_t buffer[m_length];
      if (m_serial_device->Read(reinterpret_cast<void *>(buffer), m_length))
      {
        m_data.clear();
        m_data.insert(m_data.end(), &buffer[0], &buffer[m_length]);

        m_ab.appendWithoutConversion(m_data);
        m_received_state = eRS_CHECKSUM;
      }
      break;
    }
    case eRS_CHECKSUM:
    {
      uint8_t checksum1 = 0;
      uint8_t checksum2 = 0;
      if (m_serial_device->Read(&checksum1, sizeof(uint8_t))
          && m_serial_device->Read(&checksum2, sizeof(uint8_t)))
      {
        // probe for correct checksum
        for (size_t i = 0; i < m_data.size(); ++i)
        {
          checksum1 -= m_data[i];
          checksum2 ^= m_data[i];
        }

        if ((checksum1 == 0) && (checksum2 == 0))
        {
          m_received_state = eRS_COMPLETE;
        }
        else
        {
          m_received_state = eRS_HEADER1;
        }
      }
      else
      {
        LOGGING_TRACE_C(DriverSVH, SVHReceiveThread, "Could not read checksum bytes from serial port" << endl);
      }
      break;
    }
    case eRS_COMPLETE:
    {
      // start with an empty package
      // Warning: It is imperative for correct readouts to create the received_packet with the correct length!
      SVHSerialPacket received_packet(m_length);
      m_ab >> received_packet;

      m_packets_received++;

      // notify whoever is waiting for this
      if (m_received_callback)
      {
        m_received_callback(received_packet, m_packets_received);
      }

      m_received_state = eRS_HEADER1;
      break;
    }
  }

  return true;
}

}
