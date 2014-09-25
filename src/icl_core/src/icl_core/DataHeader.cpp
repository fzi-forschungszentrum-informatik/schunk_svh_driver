// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of FZIs ic_workspace.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
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

