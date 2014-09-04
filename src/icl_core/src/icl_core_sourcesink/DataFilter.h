// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    2013-06-20
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_DATA_FILTER_TEMPLATE_H
#define ICL_CORE_SOURCESINK_DATA_FILTER_TEMPLATE_H

#include "LiveDataSource.h"
#include "DataSink.h"

namespace icl_core {
namespace sourcesink {


/** A DataFilter converts data (frames) into other data
 *
 * This is part of the DataSourceSinkFilter Framework. See Traits.h for more information.
 *
 */
template <typename source_value_type, typename sink_value_type>
class DataFilterTemplate : public LiveDataSource<source_value_type>, DataSink<sink_value_type>
{
public:
  typedef boost::shared_ptr<DataFilterTemplate> Ptr;

  /** Constructor */
  DataFilterTemplate ( const std::string &identifier = "DataFilter" );

  /** Destructor */
  virtual ~DataFilterTemplate();

  /** overload identifier function */
  virtual std::string identifier() const;

  /**
   * @brief filter
   * run the filter function on a frame and put the result into filtered_frame
   *
   * We assume that source_value_type and sink_value_type have a member "element_type". boost::shared_ptr do have this!
   * \todo make source_value_type equal the element_type and use additional boost::shared_ptr where necessary.
   *
   * @param filtered_frame The filtered frame
   * @param frame The original frame
   * @return true iff successful
   */
  virtual bool filter(typename source_value_type::element_type& filtered_frame, const typename sink_value_type::element_type& frame) = 0;

  /**
   * @brief processOnce
   *
   * \todo replace this by a processOnce-function in DataSourceSink
   *
   * This runs the filter on the data set by setNextFrame() and saves the result into the output buffer that can be accessed by frame()
   *
   * @return true iff successful
   */
//  virtual bool processOnce();

  virtual void setNextFrame(const sink_value_type &data, const Traits &traits=Traits());

  virtual source_value_type frame() const;

  /**
   * @see DataSource#seekNext
   * Map this function to processOnce();
   *
   * \todo map the function in DataSource to processOnce()
   *
   * @return always returns true
   */
  virtual bool seekNext();

};


} // namespace sourcesink
} // namespace icl_core

#include "DataFilter.hpp"

#endif // ICL_CORE_SOURCESINK_DATA_FILTER_TEMPLATE_H
