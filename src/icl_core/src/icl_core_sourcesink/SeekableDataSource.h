// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    24.04.2013
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_SEEKABLE_SOURCE_TEMPLATE_H
#define ICL_CORE_SOURCESINK_SEEKABLE_SOURCE_TEMPLATE_H

#include "Position.h"
#include "ImportExport.h"
#include <icl_core/BaseTypes.h>
#include "DataSource.h"

#include "boost/date_time.hpp"

/**
 * @file SeekableDataSource.h
 * Contains SeekableDataSource definition
 */

namespace icl_core {
namespace sourcesink {

/**
 * A SeekableDataSource reading files (videos) and providing navigation methods
 *
 * @note The following functions must be overloaded by your seekable source
 * when inheriting from a SeekableDataSource: size(), seek(), resolve(), getCurrentIndex()
 *
 * See DataSource for more details!
 *
 */
//! \note the second template parameter is not allowed to be named "Parent".
//! This would result in a Visual C++ 10 compiler error saying "Parent is no Base or Element" when running the constructor
template <class _value_type, class TParent=DataSource<_value_type> >
class SeekableDataSource : public TParent
{
public:
  typedef boost::shared_ptr<SeekableDataSource<_value_type, TParent> > Ptr;

  /** Constructor */
  /* @TODO remove redundant filename argument in Constructor and Open */
  SeekableDataSource(const std::string &identifier, const std::string &filename);

  SeekableDataSource();

  /** Destructor */
  virtual ~SeekableDataSource();

  /** Filename of the video */
  virtual std::string fileName() const;

  /**
   * @brief isSeekable
   * @return true iff source is seekable
   */
  virtual bool isSeekable() const;

  /** Returns seek(getCurrentIndex()+1) */
  virtual bool seekNext();


protected:

  void setFileName(const std::string &filename);

private:
  /** Video file name */
  std::string m_file_name;
};

} // namespace sourcesink
} // namespace icl_core

#include "SeekableDataSource.hpp"


#endif // VIDEOSOURCE_H
