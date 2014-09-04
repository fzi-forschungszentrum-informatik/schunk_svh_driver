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

#include "DataSource.h"
#include "DataSourceSink.h"
#include "DataSourceIterator.h"

namespace icl_core {
namespace sourcesink {

template <typename _value_type, typename _value_iterator>
typename DataSource<_value_type, _value_iterator>::iterator
DataSource<_value_type, _value_iterator>::begin()
{
  return iterator(this);
}

template <typename _value_type, typename _value_iterator>
typename DataSource<_value_type, _value_iterator>::iterator
DataSource<_value_type, _value_iterator>::end()
{
  return iterator();
}


template <typename _value_type, typename _value_iterator>
typename DataSource<_value_type, _value_iterator>::const_iterator
DataSource<_value_type, _value_iterator>::end() const
{
  return iterator();
}

template <typename _value_type, typename _value_iterator>
DataSource<_value_type, _value_iterator>::DataSource ( const std::string &name )
  : DataSourceBase ( name )
{
}

template <typename _value_type, typename _value_iterator>
DataSource<_value_type, _value_iterator>::~DataSource()
{
}

template <typename _value_type, typename _value_iterator>
typename DataSource<_value_type, _value_iterator>::value_type
DataSource<_value_type, _value_iterator>::nextFrame()
{
  return seekNext() ? frame() : value_type();
}


}
}
