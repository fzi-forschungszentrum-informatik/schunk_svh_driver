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
    m_skipped_bytes(0),
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
        if(!receiveData())
        {
          waitPeriod();
        }
      }
      else
      {
        LOGGING_WARNING_C(DriverSVH, SVHReceiveThread, "Cannot read data from serial device. It is not opened!" << endl);
        waitPeriod();
      }
    }

    else
    {
    // Wait for the thread period so that the timing is in sync.
    waitPeriod();
  }
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
  uint8_t data_byte;
  int bytes = m_serial_device->Read(&data_byte, sizeof(uint8_t));
  if (bytes < 0)
  {
    LOGGING_TRACE_C(DriverSVH, SVHReceiveThread, "Serial read error:" << bytes << endl );
    return false;
  }
  if (bytes < 1)
  {
    return false;
  }

  switch (m_received_state)
  {
    case eRS_HEADER1:
    {
        if (data_byte == PACKET_HEADER1)
        {
          m_received_state = eRS_HEADER2;
        }
      else
      {
        m_skipped_bytes++;
      }
      break;
    }
    case eRS_HEADER2:
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
          m_skipped_bytes++;
            break;
          }
          default:
          {
            m_received_state = eRS_HEADER1;
          m_skipped_bytes+=2;
            break;
          }
        }
      break;
    }
    case eRS_INDEX:
    {
      // Reset Array Builder for each fresh packet
      m_ab.reset(0);

        // Data bytes are not cenverted in endianess at this point
      m_ab.appendWithoutConversion(data_byte);
        m_received_state = eRS_ADDRESS;
      break;
    }
    case eRS_ADDRESS:
    {
      //get the address
      m_ab.appendWithoutConversion(data_byte);
      m_received_state = eRS_LENGTH1;
      break;
    }
    case eRS_LENGTH1:
      {
      // get payload length
      m_ab.appendWithoutConversion(data_byte);
      m_received_state = eRS_LENGTH2;
      break;
    }
    case eRS_LENGTH2:
    {
      // get payload length
      m_ab.appendWithoutConversion(data_byte);
        m_length = m_ab.readBack<uint16_t>();
        m_received_state = eRS_DATA;
      m_data.clear();
      m_data.reserve(m_length);
      break;
    }
    case eRS_DATA:
    {
      // get the payload itself
      // Some conversion due to legacy hardware calls
      m_data.push_back(data_byte);
      m_ab.appendWithoutConversion(data_byte);
      if(m_data.size()>=m_length)
      {
        m_received_state = eRS_CHECKSUM1;
      }
      break;
    }
    case eRS_CHECKSUM1:
    {
      m_checksum1 = data_byte;
      m_checksum2 = 0;
      m_received_state = eRS_CHECKSUM2;
      break;
    }
    case eRS_CHECKSUM2:
      {
      m_checksum2=data_byte;
      uint8_t checksum1=m_checksum1;
      uint8_t checksum2=m_checksum2;
        // probe for correct checksum
        for (size_t i = 0; i < m_data.size(); ++i)
        {
          checksum1 -= m_data[i];
          checksum2 ^= m_data[i];
        }

        if ((checksum1 == 0) && (checksum2 == 0))
        {
      // start with an empty package
      // Warning: It is imperative for correct readouts to create the received_packet with the correct length!
      SVHSerialPacket received_packet(m_length);
      m_ab >> received_packet;

      m_packets_received++;

        if(m_skipped_bytes>0)LOGGING_TRACE_C(DriverSVH, SVHReceiveThread, "Skipped "<<m_skipped_bytes<<" bytes "<< endl);
        LOGGING_TRACE_C(DriverSVH, SVHReceiveThread, "Received packet index:" << received_packet.index <<", address:"<<received_packet.address<<", size:"<<received_packet.data.size() << endl);
        m_skipped_bytes=0;
      // notify whoever is waiting for this
      if (m_received_callback)
      {
        m_received_callback(received_packet, m_packets_received);
      }

      m_received_state = eRS_HEADER1;
      }
      else
      {
        m_received_state = eRS_HEADER1;

        SVHSerialPacket received_packet(m_length);
        m_ab >> received_packet;

        if(m_skipped_bytes>0)LOGGING_TRACE_C(DriverSVH, SVHReceiveThread, "Skipped "<<m_skipped_bytes<<" bytes: "<< endl);
        LOGGING_TRACE_C(DriverSVH, SVHReceiveThread, "Checksum error: "<< (int)checksum1<<","<<(int)checksum2<<"!=0, skipping "<<m_length+8<<"bytes, packet index:" << received_packet.index <<", address:"<<received_packet.address<<", size:"<<received_packet.data.size() << endl);
        m_skipped_bytes=0;
        if (m_received_callback)
        {
          m_received_callback(received_packet, m_packets_received);
        }
      }
      break;
    }
  }

  return true;
}

}
