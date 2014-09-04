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

#include "Position.h"

namespace icl_core {
namespace sourcesink {

Position::Position(const FrameNumber &frame_number_) :
  index(-1), frame_number(frame_number_)
{
  // nothing to do
}

Position::Position(const boost::posix_time::ptime &timestamp_) :
  index(-1), timestamp(timestamp_), frame_number(-1)
{
  // nothing to do
}

Position::Position(const Index &index) :
  index(index), frame_number(-1)
{
  // nothing to do
}

}
}
