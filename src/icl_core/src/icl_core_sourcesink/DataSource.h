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

#ifndef ICL_CORE_SOURCESINK_DATA_SOURCE_H
#define ICL_CORE_SOURCESINK_DATA_SOURCE_H

#include "DataSourceBase.h"

#include <boost/shared_ptr.hpp>

namespace icl_core {
namespace sourcesink {



// forward declaration
template <typename _value_type>
class DataSourceIterator;

/**
 * @brief A DataSource delivers data (frames) sequentially
 *
 * See DataSourceSink for a general introduction!
 *
 * For your own implementation use one of these:
 *   DataSource<output_value_type>
 *   LiveDataSource<output_value_type>
 *   SeekableDataSource<output_value_type>
 *
 * See the specific classes for the functions that you will have to implement.
 *
 */
template <typename _value_type, typename _value_iterator = DataSourceIterator<_value_type> >
class DataSource : public DataSourceBase
{
public:
  typedef boost::shared_ptr<DataSource> Ptr;

  // STL style container interface
  typedef _value_type           value_type;
  typedef value_type &          reference;
  typedef const value_type &    const_reference;
  typedef _value_iterator       iterator;
  typedef const iterator        const_iterator;
  //typedef int difference_type;
  //typedef int size_type;

  iterator begin();
  iterator end();
  const_iterator end() const;

  /** Constructor */
  DataSource ( const std::string &identifier = "DataSource" );

  /** Destructor */
  virtual ~DataSource();

  /**
   * @brief frame
   * get the current frame.
   * @return the current frame. Usually read from a buffer.
   */
  virtual value_type frame() const = 0;

  /**
   * @brief nextFrame
   * seek to the next frame and return it.
   * runs (return seekNext() ? frame() : value_type();)
   * @return the next frame.
   */
  value_type nextFrame();

};

} // namespace sourcesink
} // namespace icl_core

#include "DataSource.hpp"

#endif // ICL_CORE_SOURCESINK_DATA_SOURCE_H
