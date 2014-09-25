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
# Try to find LibRt.  Once done, this will define:
#  LibRt_FOUND:          System has LibRt
#  LibRt_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  LibRt_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  LibRt_DEFINITIONS:    Preprocessor definitions.
#  LibRt_LIBRARIES:      only the libraries (w/o the '-l')
#  LibRt_LDFLAGS:        all required linker flags
#  LibRt_LDFLAGS_OTHER:  all other linker flags
#  LibRt_CFLAGS:         all required cflags
#  LibRt_CFLAGS_OTHER:   the other compiler flags
#  LibRt_VERSION:        version of the module
#  LibRt_PREFIX:         prefix-directory of the module
#  LibRt_INCLUDEDIR:     include-dir of the module
#  LibRt_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(LibRt LibRt
  LIBRARIES rt
  )
