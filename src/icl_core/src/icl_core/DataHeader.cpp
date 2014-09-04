// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Florian Kuhnt <kuhnt@fzi.de>
 * \date    2014-05-09
 *
 */
//----------------------------------------------------------------------
#include "DataHeader.h"

namespace icl_core {

std::ostream& operator << (std::ostream& os, StampedBase const& stamped)
{
  stamped.print(os);
  return os;
}

}

