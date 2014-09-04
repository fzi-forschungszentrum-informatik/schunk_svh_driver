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

#include "DataFilter.h"


namespace icl_core {
namespace sourcesink {

template <typename source_value_type, typename sink_value_type>
DataFilterTemplate<source_value_type, sink_value_type>::DataFilterTemplate(const std::string &identifier)
  : LiveDataSource<source_value_type>(identifier),
    DataSink<sink_value_type>(identifier)
{

}

template <typename source_value_type, typename sink_value_type>
DataFilterTemplate<source_value_type, sink_value_type>::~DataFilterTemplate()
{
}

template <typename source_value_type, typename sink_value_type>
std::string DataFilterTemplate<source_value_type, sink_value_type>::identifier() const
{
  /* use DataSourceBase.
   * As long as DataSourceSink is not virtually inherited DataSourceBase
   * and DataSinkBase hold a DataSourceSink each - we just have to select one.
   */
  return DataSourceBase::identifier();
}

template <typename source_value_type, typename sink_value_type>
void DataFilterTemplate<source_value_type, sink_value_type>::setNextFrame(const sink_value_type &data, const Traits &traits)
{
  throw Exception("not implemented this way");
}

template <typename source_value_type, typename sink_value_type>
source_value_type DataFilterTemplate<source_value_type, sink_value_type>::frame() const
{
  throw Exception("not implemented this way");
  return source_value_type();
}


template <typename source_value_type, typename sink_value_type>
bool DataFilterTemplate<source_value_type, sink_value_type>::seekNext()
{
//  return processOnce();
  return true;
}


}
}
