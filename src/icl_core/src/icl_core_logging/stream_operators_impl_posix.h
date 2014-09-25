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
 * \date    2008-03-30
 *
 * \brief   Contains ACE specific implementations of logging stream operators.
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_LOGGING_STREAM_OPERATORS_IMPL_POSIX_H_INCLUDED
#define ICL_CORE_LOGGING_STREAM_OPERATORS_IMPL_POSIX_H_INCLUDED

#include <icl_core/os_thread.h>
#include "icl_core_logging/ThreadStream.h"

namespace icl_core {
namespace logging {
//! Internal implementation details for POSIX systems.
namespace hidden_posix {

ThreadStream& operator << (ThreadStream& stream, const ThreadId& thread_id);

}
}
}

#endif
