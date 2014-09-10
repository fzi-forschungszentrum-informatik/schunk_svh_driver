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
# Try to find OsmScout.  Once done, this will define:
#  OsmScout_FOUND:          System has OsmScout
#  OsmScout_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  OsmScout_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  OsmScout_DEFINITIONS:    Preprocessor definitions.
#  OsmScout_LIBRARIES:      only the libraries (w/o the '-l')
#  OsmScout_LDFLAGS:        all required linker flags
#  OsmScout_LDFLAGS_OTHER:  all other linker flags
#  OsmScout_CFLAGS:         all required cflags
#  OsmScout_CFLAGS_OTHER:   the other compiler flags
#  OsmScout_VERSION:        version of the module
#  OsmScout_PREFIX:         prefix-directory of the module
#  OsmScout_INCLUDEDIR:     include-dir of the module
#  OsmScout_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(OsmScout libosmscout
  HEADERS osmscout/Tag.h
  LIBRARIES osmscout
  )
