# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
# -- END LICENSE BLOCK ------------------------------------------------

#----------------------------------------------------------------------
# \file
#
# \author  Jan Oberlaender <oberlaender@fzi.de>
# \date    2014-08-13
#
# Try to find Newmat.  Once done, this will define:
#  Newmat_FOUND:          System has Newmat
#  Newmat_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  Newmat_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  Newmat_DEFINITIONS:    Preprocessor definitions.
#  Newmat_LIBRARIES:      only the libraries (w/o the '-l')
#  Newmat_LDFLAGS:        all required linker flags
#  Newmat_LDFLAGS_OTHER:  all other linker flags
#  Newmat_CFLAGS:         all required cflags
#  Newmat_CFLAGS_OTHER:   the other compiler flags
#  Newmat_VERSION:        version of the module
#  Newmat_PREFIX:         prefix-directory of the module
#  Newmat_INCLUDEDIR:     include-dir of the module
#  Newmat_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(Newmat newmat
  HEADERS newmat/newmat.h
  LIBRARIES newmat
  )
