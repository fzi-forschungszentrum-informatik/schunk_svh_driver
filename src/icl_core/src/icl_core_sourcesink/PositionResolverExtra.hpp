// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    11.12.2013
*
*/
//----------------------------------------------------------------------

#include "PositionResolverExtra.h"
#include "Logger.h"

#include <algorithm>
#include <boost/filesystem.hpp>

using namespace icl_core;
using namespace sourcesink;

template <class T>
PositionResolverExtra<T>::PositionResolverExtra()
{
}

template <class T>
void PositionResolverExtra<T>::addFrame(boost::posix_time::ptime time, int frame_number, T extra_data)
{
  PositionResolver::addFrame(time, frame_number);
  m_extra_data.push_back(extra_data);
}

template <class T>
void PositionResolverExtra<T>::clear()
{
  PositionResolver::clear();
  m_extra_data.clear();
}

template <class T>
T PositionResolverExtra<T>::getExtraData(size_t index)
{
  if (index < m_extra_data.size())
  {
    return m_extra_data[index];
  }
  else
  {
    return value_type();
  }
}

template <class T>
std::vector<T> &PositionResolverExtra<T>::extraData()
{
  return m_extra_data;
}

template <class T>
const std::vector<T> &PositionResolverExtra<T>::extraData() const
{
  return m_extra_data;
}

template <class T>
void PositionResolverExtra<T>::readFromFile(std::ifstream &stream)
{
  boost::archive::text_iarchive archive(stream);
  archive >> *this;
}

template <class T>
void PositionResolverExtra<T>::writeToFile(std::ofstream &stream) const
{
  boost::archive::text_oarchive archive(stream);
  archive << *this;
}


