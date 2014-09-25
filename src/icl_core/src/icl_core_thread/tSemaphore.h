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
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2011-05-31
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_THREAD_T_SEMAPHORE_H_INCLUDED
#define ICL_CORE_THREAD_T_SEMAPHORE_H_INCLUDED

#include <icl_core/Deprecate.h>

#include "icl_core_thread/Sem.h"

namespace icl_core {
namespace thread {

typedef ICL_CORE_VC_DEPRECATE Semaphore tSemaphore ICL_CORE_GCC_DEPRECATE;

}
}

#endif
