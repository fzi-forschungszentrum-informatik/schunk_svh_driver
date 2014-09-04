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

#include "LiveDataSource.h"
#include "Exception.h"

using std::string;

namespace icl_core {
namespace sourcesink {

template <class _value_type, class Parent>
LiveDataSource<_value_type, Parent>::LiveDataSource(const std::string &identifier)
  : Parent(identifier)
{
  // nothing to do
}

template <class _value_type, class Parent>
LiveDataSource<_value_type, Parent>::~LiveDataSource()
{

}


template <class _value_type, class Parent>
bool LiveDataSource<_value_type, Parent>::isSeekable() const
{
  return false;
}

template <class _value_type, class Parent>
std::string LiveDataSource<_value_type, Parent>::fileName() const
{
  Exception e("Runnning fileName() on a LiveDataSource. This is not meant to be used and that's why it's not implemented.");
  throw e;
  return "";
}

template <class _value_type, class Parent>
int LiveDataSource<_value_type, Parent>::size() const
{
  Exception e("Runnning size() on a LiveDataSource. This is not meant to be used and that's why it's not implemented.");
  throw e;
  return -1;
}

template <class _value_type, class Parent>
bool LiveDataSource<_value_type, Parent>::seek(const Position &position)
{
  Exception e("Runnning seek() on a LiveDataSource. This is not meant to be used and that's why it's not implemented.");
  throw e;
  return false;
}

template <class _value_type, class Parent>
bool LiveDataSource<_value_type, Parent>::resolve(Position &position) const
{
  Exception e("Runnning resolve() on a LiveDataSource. This is not meant to be used and that's why it's not implemented.");
  throw e;
  return false;
}

template <class _value_type, class Parent>
int LiveDataSource<_value_type, Parent>::getCurrentIndex() const
{
  Exception e("Runnning getCurrentIndex() on a LiveDataSource. This is not meant to be used and that's why it's not implemented.");
  throw e;
  return -1;
}



} // namespace sourcesink
} // namespace icl_core
