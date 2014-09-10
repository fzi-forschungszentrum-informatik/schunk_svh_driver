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
# Try to find OsmScoutMap.  Once done, this will define:
#  OsmScoutMap_FOUND:          System has OsmScoutMap
#  OsmScoutMap_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  OsmScoutMap_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  OsmScoutMap_DEFINITIONS:    Preprocessor definitions.
#  OsmScoutMap_LIBRARIES:      only the libraries (w/o the '-l')
#  OsmScoutMap_LDFLAGS:        all required linker flags
#  OsmScoutMap_LDFLAGS_OTHER:  all other linker flags
#  OsmScoutMap_CFLAGS:         all required cflags
#  OsmScoutMap_CFLAGS_OTHER:   the other compiler flags
#  OsmScoutMap_VERSION:        version of the module
#  OsmScoutMap_PREFIX:         prefix-directory of the module
#  OsmScoutMap_INCLUDEDIR:     include-dir of the module
#  OsmScoutMap_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(OsmScoutMap libosmscout-map
  HEADERS osmscout/MapPainter.h
  LIBRARIES osmscoutmap
  )
