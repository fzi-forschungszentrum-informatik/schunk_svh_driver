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
# Try to find LibArchive.  Once done, this will define:
#  LibArchive_FOUND:          System has LibArchive
#  LibArchive_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  LibArchive_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  LibArchive_DEFINITIONS:    Preprocessor definitions.
#  LibArchive_LIBRARIES:      only the libraries (w/o the '-l')
#  LibArchive_LDFLAGS:        all required linker flags
#  LibArchive_LDFLAGS_OTHER:  all other linker flags
#  LibArchive_CFLAGS:         all required cflags
#  LibArchive_CFLAGS_OTHER:   the other compiler flags
#  LibArchive_VERSION:        version of the module
#  LibArchive_PREFIX:         prefix-directory of the module
#  LibArchive_INCLUDEDIR:     include-dir of the module
#  LibArchive_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(LibArchive libarchive
  HEADERS archive.h
  LIBRARIES archive
  )
