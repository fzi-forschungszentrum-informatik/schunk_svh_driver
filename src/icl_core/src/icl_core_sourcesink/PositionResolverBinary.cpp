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

#include "PositionResolverBinary.h"
#include "Logger.h"

#include <algorithm>
#include <boost/filesystem.hpp>

using namespace icl_core;
using namespace sourcesink;

PositionResolverBinary::PositionResolverBinary()
{
}

void PositionResolverBinary::addFrame(boost::posix_time::ptime time, int frame_number, uint64_t binary_position)
{
  PositionResolver::addFrame(time, frame_number);
  m_binary_positions.push_back(binary_position);
}

void PositionResolverBinary::clear()
{
  PositionResolver::clear();
  m_binary_positions.clear();
}

PositionResolverBinary::BinaryPosition PositionResolverBinary::getBinaryPosition(size_t index)
{
  if (index < m_binary_positions.size())
  {
    return m_binary_positions[index];
  }
  else
  {
    return std::numeric_limits<BinaryPosition>::max();
  }
}

std::vector<uint64_t> &PositionResolverBinary::binaryPositions()
{
  return m_binary_positions;
}

const std::vector<uint64_t> &PositionResolverBinary::binaryPositions() const
{
  return m_binary_positions;
}

void PositionResolverBinary::readFromFile(std::ifstream &stream)
{
  boost::archive::text_iarchive archive(stream);
  archive >> *this;
}

void PositionResolverBinary::writeToFile(std::ofstream &stream) const
{
  boost::archive::text_oarchive archive(stream);
  archive << *this;
}


