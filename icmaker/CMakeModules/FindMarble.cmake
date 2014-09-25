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
# Try to find Marble.  Once done, this will define:
#  Marble_FOUND:          System has Marble
#  Marble_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  Marble_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  Marble_DEFINITIONS:    Preprocessor definitions.
#  Marble_LIBRARIES:      only the libraries (w/o the '-l')
#  Marble_LDFLAGS:        all required linker flags
#  Marble_LDFLAGS_OTHER:  all other linker flags
#  Marble_CFLAGS:         all required cflags
#  Marble_CFLAGS_OTHER:   the other compiler flags
#  Marble_VERSION:        version of the module
#  Marble_PREFIX:         prefix-directory of the module
#  Marble_INCLUDEDIR:     include-dir of the module
#  Marble_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(Marble marble
  HEADERS marble/MarbleMap.h
  LIBRARIES marblewidget
  )
