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
 * \date    2008-03-29
 *
 */
//----------------------------------------------------------------------
#include "icl_core/os_posix_fs.h"

#include <stdio.h>
#include <unistd.h>

namespace icl_core {
namespace os {
namespace hidden_posix {

int rename(const char *old_filename, const char *new_filename)
{
  return ::rename(old_filename, new_filename);
}

int unlink(const char *filename)
{
  return ::unlink(filename);
}

}
}
}
