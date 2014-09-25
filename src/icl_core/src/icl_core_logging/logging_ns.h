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
 * \date    2008-01-28
 *
 * \brief   Definition of the implementation namespace
 *          for global functions.
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_LOGGING_LOGGING_NS_H_INCLUDED
#define ICL_CORE_LOGGING_LOGGING_NS_H_INCLUDED

#if defined _SYSTEM_POSIX_
# define ICL_CORE_LOGGING_IMPL_NS ::icl_core::logging::hidden_posix
#elif defined _SYSTEM_WIN32_
# define ICL_CORE_LOGGING_IMPL_NS ::icl_core::logging::hidden_win32
#else
# error "No ::icl_core::logging namespace implementation defined for this platform."
#endif

#endif
