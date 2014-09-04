// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    08.04.2013
*
*/
//----------------------------------------------------------------------

#include "DataHeader.h"


namespace icl_core {
namespace sourcesink {


DataHeader::DataHeader(const uint32_t id, const boost::posix_time::ptime &timestamp, const std::string &coordinate_system)
{
  setId(id);
  setTimestamp(timestamp);
  setCoordinateSystem(coordinate_system);
}

void DataHeader::setId(const uint32_t id)
{
  m_id = id;
}

uint32_t DataHeader::getId() const
{
  return m_id;
}

uint32_t &DataHeader::id()
{
  return m_id;
}

const uint32_t DataHeader::id() const
{
  return m_id;
}

void DataHeader::setTimestamp(const boost::posix_time::ptime &timestamp)
{
  m_timestamp = timestamp;
}

boost::posix_time::ptime DataHeader::getTimestamp() const
{
  return m_timestamp;
}

boost::posix_time::ptime &DataHeader::timestamp()
{
  return m_timestamp;
}

const boost::posix_time::ptime &DataHeader::timestamp() const
{
  return m_timestamp;
}

bool DataHeader::hasTimestamp() const
{
  return !m_timestamp.is_not_a_date_time();
}

void DataHeader::setCoordinateSystem(const std::string &coordinate_system)
{
  m_coordinate_system = coordinate_system;
}

std::string DataHeader::getCoordinateSystem() const
{
  return m_coordinate_system;
}

std::string &DataHeader::coordinateSystem()
{
  return m_coordinate_system;
}

const std::string &DataHeader::coordinateSystem() const
{
  return m_coordinate_system;
}


}
}
