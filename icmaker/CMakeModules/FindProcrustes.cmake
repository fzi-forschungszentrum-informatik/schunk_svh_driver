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
# Try to find Procrustes.  Once done, this will define:
#  Procrustes_FOUND:          System has Procrustes
#  Procrustes_INCLUDE_DIRS:   The '-I' preprocessor flags (w/o the '-I')
#  Procrustes_LIBRARY_DIRS:   The paths of the libraries (w/o the '-L')
# Variables defined if pkg-config was employed:
#  Procrustes_DEFINITIONS:    Preprocessor definitions.
#  Procrustes_LIBRARIES:      only the libraries (w/o the '-l')
#  Procrustes_LDFLAGS:        all required linker flags
#  Procrustes_LDFLAGS_OTHER:  all other linker flags
#  Procrustes_CFLAGS:         all required cflags
#  Procrustes_CFLAGS_OTHER:   the other compiler flags
#  Procrustes_VERSION:        version of the module
#  Procrustes_PREFIX:         prefix-directory of the module
#  Procrustes_INCLUDEDIR:     include-dir of the module
#  Procrustes_LIBDIR:         lib-dir of the module
#----------------------------------------------------------------------

include(PrintLibraryStatus)
include(LibFindMacros)

libfind_lib_with_pkg_config(Procrustes procrustes
  HEADERS Procrustes/qfMahalanobis.hpp
  )
