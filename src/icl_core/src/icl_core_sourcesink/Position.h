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

#ifndef ICL_CORE_SOURCESINK_POSITION_H
#define ICL_CORE_SOURCESINK_POSITION_H

#include "boost/date_time.hpp"
#include "icl_core_sourcesink/ImportExport.h"

namespace icl_core {
namespace sourcesink {

/**
 * An integer with an explicit constructor to enforce awareness between
 * the difference of 'int index' and 'int frame_number'
 */
template<class T, class U>
class Explicit
{
public:
  explicit Explicit(T value_=-1) : value(value_)
  {
    // nothing to do
  }

  operator T() const
  {
    return value;
  }

  T value;
};

typedef Explicit<int, int> Index;
typedef Explicit<int, double> FrameNumber;

/**
 * Tuple to store position information for one frame:
 * Index (first is 0, neighboring frames have a difference of 1),
 * frame number (monotonic increasing, but arbitrary numbers allowed otherwise,
 * especially the first frame can be != 0 and a difference larger than 1 between
 * neighboring frames,
 * timestamp, similar behavior to frame number
 */
class ICL_CORE_SOURCESINK_IMPORT_EXPORT Position
{
public:
  Position(const FrameNumber &frame_number);
  Position(const boost::posix_time::ptime &timestamp);
  Position(const Index &index);

  Index index;
  boost::posix_time::ptime timestamp;
  FrameNumber frame_number;
};

}
}

#endif // ICL_CORE_SOURCESINK_POSITION_H
