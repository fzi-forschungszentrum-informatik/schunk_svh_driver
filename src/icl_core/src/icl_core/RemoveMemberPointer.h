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
 * \date    2012-01-24
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_REMOVE_MEMBER_POINTER_H_INCLUDED
#define ICL_CORE_REMOVE_MEMBER_POINTER_H_INCLUDED

namespace icl_core {

template <typename T>
struct RemoveMemberPointer
{
  typedef T Type;
};

template <typename T, class Q>
struct RemoveMemberPointer<T Q::*>
{
  typedef T Type;
};

}

#endif
