// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2013-10-23
*
*/
//----------------------------------------------------------------------

#include "DataSourceBase.h"
#include "Exception.h"

namespace icl_core {
namespace sourcesink {

using namespace std;


DataSourceBase::DataSourceBase(const std::string &identifier)
  : DataSourceSink(identifier)
{
}

DataSourceBase::~DataSourceBase()
{
  // nothing to do
}

boost::posix_time::ptime DataSourceBase::getCurrentTimestamp() const
{
  Position pos = Position(Index(getCurrentIndex()));
  resolve(pos);
  return pos.timestamp;
}

boost::posix_time::ptime DataSourceBase::getNextTime() const
{
  Position pos = Position(Index(getCurrentIndex() + 1));
  resolve(pos);
  return pos.timestamp;
}

boost::posix_time::ptime DataSourceBase::getFirstTime() const
{
  if (isSeekable())
  {
    Position pos = Position(Index(0));
    resolve(pos);
    return pos.timestamp;
  }
  else
  {
    return boost::posix_time::not_a_date_time;
  }
}

boost::posix_time::ptime DataSourceBase::getLastTime() const
{
  if (isSeekable())
  {
    Position pos(Index(size()-1));
    resolve(pos);
    return pos.timestamp;
  }
  else
  {
    return boost::posix_time::not_a_date_time;
  }
}

bool DataSourceBase::isTimestampValid(const boost::posix_time::ptime &timestamp) const
{
  return timestamp >= getFirstTime() && timestamp <= getLastTime();
}

void DataSourceBase::seekRelative(int index_offset)
{
  seek(Index(getCurrentIndex() + index_offset));
}

boost::posix_time::ptime DataSourceBase::getTimestamp(const int &frameNumber) const
{
  Position target = Position(FrameNumber(frameNumber));
  this->resolve(target);
  return target.timestamp;
}

int DataSourceBase::getFramenumber(const boost::posix_time::ptime &timestamp) const
{
  Position target(timestamp);
  this->resolve(target);
  return target.frame_number;
}

}
}
