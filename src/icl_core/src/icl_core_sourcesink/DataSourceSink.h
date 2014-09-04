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

#ifndef ICL_CORE_SOURCESINK_DATASOURCESINK_H
#define ICL_CORE_SOURCESINK_DATASOURCESINK_H

#include "ImportExport.h"
#include <boost/shared_ptr.hpp>
#include <string>

namespace icl_core {
namespace sourcesink {


/**
 * @brief The DataSourceSink class
 *
 * This is the basis of the DataSourceSinkFilter Framework.
 *
 *
 * \todo write introduction to whole framework
 *
 *
 *
 *  main features of the classes are:
 *
 *  DataSource
 *           processOnce()  (= seekNext)
 *           frame()
 *           nextFrame()  (= processOnce, frame)
 *
 *  DataSink
 *           setFrame()
 *           processOnce()
 *           setNextFrame()  (= setFrame, processOnce)
 *
 *  DataFilter
 *           setFrame()
 *           processOnce()   (= filter(out_buffer, in_buffer))
 *           frame()
 *           filter(&out &in)
 *
 * Use DataSource<output_value_type>, LiveDataSource<output_value_type>, SeekableDataSource<output_value_type>,
 * DataSink<input_value_type> and DataFilter<output_value_type, input_value_type> for your own implementations!
 *
 *
 *
 */
class ICL_CORE_SOURCESINK_IMPORT_EXPORT DataSourceSink
{
public:
  /** Constructor */
  DataSourceSink ( const std::string &identifier = "DataSourceSink" );

  virtual ~DataSourceSink();

  /**
   * @brief identifier
   * @return the identifier string of the current source or sink
   */
  virtual std::string identifier() const;

  /**
   * @brief processOnce
   *
   * \todo make this function pure virtual
   *
   * This runs one step of the source, sink or filter: Loads the next image, writes the current image or just processes the filter on the current input data.
   *
   * @return true iff successful
   */
  virtual bool processOnce(); // = 0;

private:
  DataSourceSink(const DataSourceSink&);                 // Prevent copy-construction
  DataSourceSink& operator=(const DataSourceSink&);      // Prevent assignment

  /** source/sink specifier */
  std::string m_identifier;
};

}
}

#endif // ICL_CORE_SOURCESINK_DATASOURCESINK_H
