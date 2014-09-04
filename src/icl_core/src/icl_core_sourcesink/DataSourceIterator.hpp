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

#include "DataSourceIterator.h"


namespace icl_core {
namespace sourcesink {

template <typename _vT>
DataSourceIterator<_vT>::DataSourceIterator(DataSource<_vT, DataSourceIterator<_vT> > *source) :
  m_data_source(source)
{
  updateData();
}

template <typename _vT>
DataSourceIterator<_vT> &DataSourceIterator<_vT>::operator ++()
{
  updateData();
  return *this;
}

template <typename _vT>
bool DataSourceIterator<_vT>::operator ==(const DataSourceIterator &other)
{
  return m_data_source == other.m_data_source;
}

template <typename _vT>
bool DataSourceIterator<_vT>::operator !=(const DataSourceIterator &other)
{
  return(!((*this) == other));
}

template <typename _vT>
typename DataSourceIterator<_vT>::reference DataSourceIterator<_vT>::operator *()
{
  return m_data;
}

template <typename _vT>
void DataSourceIterator<_vT>::updateData()
{
  if (m_data_source)
  {
    m_data = m_data_source->nextFrame();
    if (!m_data)
    {
      m_data_source = 0; // eof
    }
  }
}

}
}
