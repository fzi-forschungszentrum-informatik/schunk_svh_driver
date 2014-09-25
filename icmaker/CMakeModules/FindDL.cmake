# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
#
# This file is part of the icmaker build system.
#
# This program is free software licensed under the BSD License. You can
# find a copy of this license in the LICENSE folder in the top directory
# of the source code.
#
# © Copyright 2013 FZI Forschungszentrum Informatik, Karlsruhe, Germany
#
# -- END LICENSE BLOCK ------------------------------------------------

#----------------------------------------------------------------------
# \file
#
# \author  Jan Oberlaender <oberlaender@fzi.de>
# \date    2014-08-13
#
# Try to find DL.  Once done, this will define:
#  DL_FOUND:          System has DL
#  DL_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  DL_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  DL_DEFINITIONS:    Preprocessor definitions.
#  DL_LIBRARIES:      only the libraries (w/o the '-l')
#  DL_LDFLAGS:        all required linker flags
#  DL_LDFLAGS_OTHER:  all other linker flags
#  DL_CFLAGS:         all required cflags
#  DL_CFLAGS_OTHER:   the other compiler flags
#  DL_VERSION:        version of the module
#  DL_PREFIX:         prefix-directory of the module
#  DL_INCLUDEDIR:     include-dir of the module
#  DL_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(DL libdl
  HEADERS dlfcn.h
  LIBRARIES dl
  )
