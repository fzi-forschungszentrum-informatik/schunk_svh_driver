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
 * \author  Jan Oberländer <oberlaen@fzi.de>
 * \date    2010-06-16
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_LOGGING_T_SCOPED_SEMAPHORE_H_INCLUDED
#define ICL_CORE_LOGGING_T_SCOPED_SEMAPHORE_H_INCLUDED

#include "icl_core/Deprecate.h"
#include "icl_core_logging/ScopedSemaphore.h"

namespace icl_core {
namespace logging {

typedef ICL_CORE_VC_DEPRECATE ScopedSemaphore tScopedSemaphore ICL_CORE_GCC_DEPRECATE;

}
}

#endif
