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
# Try to find Eigen3.  Once done, this will define:
#  Eigen3_FOUND:          System has Eigen3
#  Eigen3_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  Eigen3_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  Eigen3_DEFINITIONS:    Preprocessor definitions.
#  Eigen3_LIBRARIES:      only the libraries (w/o the '-l')
#  Eigen3_LDFLAGS:        all required linker flags
#  Eigen3_LDFLAGS_OTHER:  all other linker flags
#  Eigen3_CFLAGS:         all required cflags
#  Eigen3_CFLAGS_OTHER:   the other compiler flags
#  Eigen3_VERSION:        version of the module
#  Eigen3_PREFIX:         prefix-directory of the module
#  Eigen3_INCLUDEDIR:     include-dir of the module
#  Eigen3_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(Eigen3 eigen3
  HEADERS Eigen/Core
  HEADER_PATHS "/usr/include/eigen3" "${CMAKE_INSTALL_PREFIX}/include/eigen3"
  DEFINE _IC_BUILDER_EIGEN_
  )

