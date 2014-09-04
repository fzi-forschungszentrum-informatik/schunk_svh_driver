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

#include "Traits.h"

namespace icl_core {
namespace sourcesink {

DataSinkBase::DataSinkBase(const std::string &identifier)
  : DataSourceSink(identifier)
{
}

DataSinkBase::~DataSinkBase()
{
  // nothing to do
}


Traits::Traits(DataSourceBase *source, Traits *parent) :
  m_data_source(source), m_parent(parent)
{
  // nothing to do
}

DataSourceBase *Traits::dataSource() const
{
  return m_data_source;
}

Traits *Traits::parent() const
{
  return m_parent;
}

const Traits *Traits::root() const
{
  const Traits* origin = this;
  for (; origin->dataSource() && origin->parent(); origin=origin->parent()); // loop has no inner part
  return origin;
}



}
}
