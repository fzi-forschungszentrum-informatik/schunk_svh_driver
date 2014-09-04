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

#ifndef ICL_CORE_SOURCESINK_DATAHEADER_H
#define ICL_CORE_SOURCESINK_DATAHEADER_H

#include "ImportExport.h"

#include <icl_core/BaseTypes.h>
#include <boost/date_time.hpp>
#include <string>


namespace icl_core {
namespace sourcesink {

class ICL_CORE_SOURCESINK_IMPORT_EXPORT DataHeader
{
public:
  DataHeader(const uint32_t id = -1, const boost::posix_time::ptime &timestamp = boost::posix_time::ptime(), const std::string &coordinate_system = "");


  void setId(const uint32_t id);
  uint32_t getId() const;
  uint32_t& id();
  const uint32_t id() const;

  void setTimestamp(const boost::posix_time::ptime &timestamp);
  boost::posix_time::ptime getTimestamp() const;
  boost::posix_time::ptime& timestamp();
  const boost::posix_time::ptime& timestamp() const;

  bool hasTimestamp() const;

  void setCoordinateSystem(const std::string &coordinate_system);
  std::string getCoordinateSystem() const;
  std::string& coordinateSystem();
  const std::string& coordinateSystem() const;

private:

  uint32_t m_id;

  boost::posix_time::ptime m_timestamp;

  std::string m_coordinate_system;
};


} // namespace sourcesink
} // namespace icl_core


#endif // ICL_CORE_SOURCESINK_DATAHEADER_H
