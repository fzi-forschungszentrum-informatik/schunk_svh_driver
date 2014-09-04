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

#include "SeekableDataSource.h"
#include "Exception.h"

using std::string;

namespace icl_core {
namespace sourcesink {

template <class _value_type, class TParent>
SeekableDataSource<_value_type, TParent>::SeekableDataSource(const std::string &name, const std::string &filename)
  : TParent(name),
    m_file_name(filename)
{
  // nothing to do
}

template <class _value_type, class TParent>
SeekableDataSource<_value_type, TParent>::SeekableDataSource()
{
}

template <class _value_type, class TParent>
SeekableDataSource<_value_type, TParent>::~SeekableDataSource()
{
  // nothing to do
}

template <class _value_type, class TParent>
string SeekableDataSource<_value_type, TParent>::fileName() const
{
  return m_file_name;
}

template <class _value_type, class TParent>
bool SeekableDataSource<_value_type, TParent>::isSeekable() const
{
  return true;
}

template <class _value_type, class TParent>
bool SeekableDataSource<_value_type, TParent>::seekNext()
{
  return this->seek(Index(this->getCurrentIndex()+1));
}

template <class _value_type, class TParent>
void SeekableDataSource<_value_type, TParent>::setFileName(const string &filename)
{
  m_file_name = filename;
}

} // namespace sourcesink
} // namespace icl_core
