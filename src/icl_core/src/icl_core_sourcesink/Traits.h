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

#ifndef ICL_CORE_SOURCESINK_TRAITS_H
#define ICL_CORE_SOURCESINK_TRAITS_H


#include "DataSourceSink.h"
#include "DataSourceBase.h"
#include "SeekableDataSource.h"
#include "ImportExport.h"
#include <boost/shared_ptr.hpp>
#include <string>

namespace icl_core {
//! Source/sink framework.
namespace sourcesink {

/**
 * @brief The DataSinkBase class is the base class for DataSinks.
 *
 * It is independent of the data type.
 *
 */
class ICL_CORE_SOURCESINK_IMPORT_EXPORT DataSinkBase : public DataSourceSink
{
public:
  typedef boost::shared_ptr<DataSinkBase> Ptr;

  DataSinkBase ( const std::string &identifier = "DataSink" );

  virtual ~DataSinkBase();
};


/**
 * @brief The Traits class holds a source-sink-chain.
 *
 * It is commonly used when giving data to a DataSink via setNextFrame() so that
 * the Sink knows about the Source and can seek and do other stuff on the source.
 */
//! \todo replace all pointer by shared_ptr?
class ICL_CORE_SOURCESINK_IMPORT_EXPORT Traits
{
public:
  explicit Traits(DataSourceBase* source=0, Traits* parent=0);

  /**
   * @brief dataSource returns a dataSource pointer to this Traits' source.
   * @return
   */
  DataSourceBase* dataSource() const;

  /**
   * @brief parent returns a pointer to the parent Traits object.
   * @return
   */
  Traits* parent() const;

  /**
   * @brief root returns a pointer to the root Traits object by recursively
   * travelling through Traits.
   *
   * @return
   */
  const Traits* root() const;

  /**
   * @brief as casts the dataSource to a requested source type.
   * @return
   */
  template<typename T>
  T* as() const
  {
    return dynamic_cast<T*>(m_data_source);
  }

private:
  DataSourceBase* m_data_source;
  Traits* m_parent;
};

} // namespace sourcesink
} // namespace icl_core


#endif // ICL_CORE_SOURCESINK_TRAITS_H
