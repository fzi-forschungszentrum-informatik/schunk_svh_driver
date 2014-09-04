// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2013-10-23
*
*/
//----------------------------------------------------------------------

#include "DataSourceSink.h"

#include "Exception.h"

namespace icl_core {
namespace sourcesink {

DataSourceSink::DataSourceSink(const std::string &identifier)
  : m_identifier ( identifier )
{
}

DataSourceSink::~DataSourceSink()
{
}

std::string DataSourceSink::identifier() const
{
  return m_identifier;
}

bool DataSourceSink::processOnce()
{
  throw Exception("dummy implementation - will be removed in future");
  return true;
}


}
}
