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




#ifndef ICL_CORE_SOURCESINK_DATA_SINK_H
#define ICL_CORE_SOURCESINK_DATA_SINK_H

#include "Traits.h"

#include <boost/shared_ptr.hpp>
#include <string>

namespace icl_core {
namespace sourcesink {


/**
 * A DataSink processes data (frames) sequentially, saving them to a
 * file (or files) or displays them on some device
 * 
 * This is part of the DataSourceSinkFilter Framework. See Traits.h for more information.
 *
 */
template <typename _value_type>
class DataSink : public DataSinkBase
{
public:
  typedef boost::shared_ptr<DataSink> Ptr;

  // STL style container interface
  typedef _value_type                    value_type;
  typedef value_type &                    reference;
  typedef const value_type &              const_reference;

  /** Constructor */
  DataSink( const std::string &identifier = "DataSink" );

  /** Destructor */
  virtual ~DataSink();

  /**
    * Set the next data (frame)
    * Will be implemented by the sinks.
    */
  /**
   * @brief setNextFrame
   *
   * Set the next data (frame)
   * Will be implemented by the sinks.
   *
   * \todo replace this function by a function calling setFrame() and processOnce() similar to DataSource#nextFrame().
   *
   * @param data The new data that should be set.
   * @param traits A Traits object representing the source of the data.
   */
  virtual void setNextFrame(const value_type &data, const Traits &traits=Traits()) = 0;

  /**
   * @brief setFrame
   *
   * Set the next data (frame)
   * It only sets the data - commonly writing the data into a buffer - and doesn't process the data.
   * Use processOnce() afterwards to process the data.
   *
   * \todo make this function pure virtual
   *
   * @param data The new data that should be set.
   * @param traits A Traits object representing the source of the data.
   */
  virtual void setFrame(const value_type &data, const Traits &traits=Traits()); // = 0;


};


} // namespace sourcesink
} // namespace icl_core

#include "DataSink.hpp"

#endif // ICL_CORE_SOURCESINK_DATA_SINK_H
