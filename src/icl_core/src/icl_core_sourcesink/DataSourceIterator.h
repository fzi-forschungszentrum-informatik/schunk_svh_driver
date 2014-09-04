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

#ifndef ICL_CORE_SOURCESINK_DATA_SOURCE_ITERATOR_H
#define ICL_CORE_SOURCESINK_DATA_SOURCE_ITERATOR_H

#include <iterator>
#include <boost/shared_ptr.hpp>
#include <string>

namespace icl_core {
namespace sourcesink {

// forward declaration
template <typename _value_type, typename _value_iterator>
class DataSource;

/**
 * Reads data from a data source, with iterator style access
 */
template <typename _value_type>
class DataSourceIterator : public std::iterator< std::input_iterator_tag ,
    const _value_type , int >
{
public:
  /// The type "pointed to" by the iterator.
  typedef _value_type        value_type;
  /// This type represents a reference-to-value_type.
  typedef value_type& reference;

  DataSourceIterator(DataSource<value_type, DataSourceIterator<value_type> >* source=0);

  DataSourceIterator& operator++();

  bool operator==( const DataSourceIterator& other );

  bool operator!=(const DataSourceIterator& other);

  reference operator*();

  reference operator->(){
    /** @todo implement */
    return (*(*this));
  }

private:
  void updateData();

  DataSource<value_type, DataSourceIterator<value_type> >* m_data_source;
  _value_type m_data;
};


} // namespace sourcesink
} // namespace icl_core

#include "DataSourceIterator.hpp"

#endif // ICL_CORE_SOURCESINK_DATA_SOURCE_ITERATOR_H
