// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2013-10-16
*
*/
//----------------------------------------------------------------------

#include "PositionResolver.h"
#include "Logger.h"

#include <algorithm>
#include <boost/filesystem.hpp>

using namespace icl_core;
using namespace sourcesink;

const std::string PositionResolver::TYPICAL_FILE_SUFFIX = ".index.cache";

PositionResolver::PositionResolver()
{
}

bool PositionResolver::resolve(Position &position) const
{
  //Check if there is a valid index
  if (position.index >= 0)
  {
    if( std::size_t(position.index) >= m_frame_numbers.size())
    {
      SOURCE_SINK_WARN("Position index (" << position.index << ") is out of bounds. Maximal index is " << ((int)m_frame_numbers.size()) - 1);
      return false;
    }

    position.timestamp = m_timestamps[position.index];
    position.frame_number = FrameNumber(m_frame_numbers[position.index]);
    return true;
  }

  //Check if there is a valid frame number
  if (position.frame_number >= 0)
  {
    std::vector<int>::const_iterator frame_iter = std::upper_bound(m_frame_numbers.begin(), m_frame_numbers.end(), position.frame_number);

    if(frame_iter == m_frame_numbers.end())
    {
      SOURCE_SINK_WARN("Position framenumber (" << position.frame_number << ") is out of bounds. Maximal framenumber is " << m_frame_numbers.back());
      return false;
    }

    --frame_iter;

    position.index = Index(frame_iter - m_frame_numbers.begin());
    position.timestamp = m_timestamps[position.index];
    position.frame_number = FrameNumber(m_frame_numbers[position.index]);
    return true;
  }

  //Check if there is a valid timestamp
  if (!position.timestamp.is_special())
  {
    std::vector<boost::posix_time::ptime>::const_iterator time_iter = std::upper_bound(m_timestamps.begin(), m_timestamps.end(), position.timestamp);

    if(time_iter == m_timestamps.end())
    {
      SOURCE_SINK_WARN("Position timestamp (" << to_simple_string(position.timestamp)
                       << ") is out of bounds. Maximal time is " << to_simple_string(m_timestamps.back()));
      return false;
    }
    --time_iter;

    position.index = Index(time_iter - m_timestamps.begin());
    position.timestamp = m_timestamps[position.index];
    position.frame_number = FrameNumber(m_frame_numbers[position.index]);
    return true;
  }

  return false;
}

void PositionResolver::addFrame(boost::posix_time::ptime time, int frame_number)
{
  m_timestamps.push_back(time);
  m_frame_numbers.push_back(frame_number);
}

size_t PositionResolver::size() const
{
  return m_timestamps.size();
}

void PositionResolver::clear()
{
  m_timestamps.clear();
  m_frame_numbers.clear();
}

bool PositionResolver::load(const std::string &filename)
{
  if (!boost::filesystem::exists(filename))
  {
    return false;
  }

  try
  {
    std::ifstream stream(filename.c_str(), std::ios::binary);
    readFromFile(stream);
    return true;
  }
  catch (boost::archive::archive_exception &ex)
  {
    SOURCE_SINK_WARN("Failed to open " << filename << ": " << ex.what());
    return false;
  }
}

bool PositionResolver::save(const std::string &filename) const
{
  try
  {
    std::ofstream stream(filename.c_str());
    writeToFile(stream);
    return true;
  }
  catch (boost::archive::archive_exception &ex)
  {
    SOURCE_SINK_WARN("Failed to write " << filename << ": " << ex.what());
    return false;
  }
}

std::vector<boost::posix_time::ptime> &PositionResolver::timestamps()
{
  return m_timestamps;
}

const std::vector<boost::posix_time::ptime> &PositionResolver::timestamps() const
{
  return m_timestamps;
}

std::vector<int> &PositionResolver::frameNumbers()
{
  return m_frame_numbers;
}

const std::vector<int> &PositionResolver::frameNumbers() const
{
  return m_frame_numbers;
}

void PositionResolver::readFromFile(std::ifstream &stream)
{
  boost::archive::text_iarchive archive(stream);
  archive >> *this;
}

void PositionResolver::writeToFile(std::ofstream& stream) const
{
  boost::archive::text_oarchive archive(stream);
  archive << *this;
}

