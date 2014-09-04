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

#include "DataSink.h"


namespace icl_core {
namespace sourcesink {

template <typename _value_type>
DataSink<_value_type>::DataSink (const std::string &identifier ) :
  DataSinkBase(identifier)
{
    // nothing to do
}

template <typename _value_type>
DataSink<_value_type>::~DataSink()
{
  // nothing to do
}

template <typename _value_type>
void DataSink<_value_type>::setFrame(const _value_type &data, const Traits &traits)
{
  throw Exception("dummy implementation - will be removed in future");
}


}
}
