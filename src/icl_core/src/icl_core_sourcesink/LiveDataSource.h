// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    30.10.2013
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_LIVE_DATA_SOURCE_TEMPLATE_H
#define ICL_CORE_SOURCESINK_LIVE_DATA_SOURCE_TEMPLATE_H

#include "Position.h"
#include "ImportExport.h"
#include <icl_core/BaseTypes.h>
#include "DataSource.h"

#include "boost/date_time.hpp"

/**
 * @file LiveDataSource.h
 * Contains SeekableDataSource definition
 */

namespace icl_core {
namespace sourcesink {

/**
 * A LiveDataSource implements a DataSource usable for Live Sensors
 *
 * @note The following functions must be overloaded by your live source
 * when inheriting from a LiveDataSource: frame(), seekNext()
 *
 * See DataSource for more details!
 *
 */
template <class _value_type, class Parent=DataSource<_value_type, DataSourceIterator<_value_type> > >
class LiveDataSource : public Parent
// removed the ICL_CORE_SOURCESINK_IMPORT_EXPORT since windows complaint it is not allowed
// class ICL_CORE_SOURCESINK_IMPORT_EXPORT LiveDataSource : public Parent 
{
public:
  typedef boost::shared_ptr<LiveDataSource<_value_type, Parent> > Ptr;

  /** Constructor */
  LiveDataSource(const std::string &identifier = "LiveDataSource");

  /** Destructor */
  virtual ~LiveDataSource();


  // default to non-seekable
  virtual bool isSeekable() const;



  // dummy-implement functions that we won't use

  //! \todo can these be set to private?

  virtual std::string fileName() const;
  virtual int size() const;
  virtual bool seek(const Position &position);
  virtual bool resolve(Position &position) const;
  virtual int getCurrentIndex() const;

};

} // namespace sourcesink
} // namespace icl_core

#include "LiveDataSource.hpp"


#endif // VIDEOSOURCE_H
