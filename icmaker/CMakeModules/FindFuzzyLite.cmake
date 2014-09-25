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
# Try to find FuzzyLite.  Once done, this will define:
#  FuzzyLite_FOUND:          System has FuzzyLite
#  FuzzyLite_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  FuzzyLite_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  FuzzyLite_DEFINITIONS:    Preprocessor definitions.
#  FuzzyLite_LIBRARIES:      only the libraries (w/o the '-l')
#  FuzzyLite_LDFLAGS:        all required linker flags
#  FuzzyLite_LDFLAGS_OTHER:  all other linker flags
#  FuzzyLite_CFLAGS:         all required cflags
#  FuzzyLite_CFLAGS_OTHER:   the other compiler flags
#  FuzzyLite_VERSION:        version of the module
#  FuzzyLite_PREFIX:         prefix-directory of the module
#  FuzzyLite_INCLUDEDIR:     include-dir of the module
#  FuzzyLite_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(FuzzyLite libgps
  HEADERS fuzzylite/FuzzyLite.h
  LIBRARIES fuzzylite
  HINTS /opt/local /opt/tools/fuzzylite-i386
  )

