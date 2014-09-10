# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This file is part of the SCHUNK SVH Driver suite.
#
# This program is free software licensed under the LGPL
# (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
# You can find a copy of this license in LICENSE.txt in the top
# directory of the source code.
#
# © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
# © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
#
# -- END LICENSE BLOCK ------------------------------------------------

# - Try to find SerialConverter Driver LibFtd
# Once done this will define
#  ftd2xx_FOUND - System has ftd2xx
#  ftd2xx_INCLUDE_DIRS - The ftd2xx include directories
#  ftd2xx_LIBRARIES - link these to use ftd2xx

include(PrintLibraryStatus)
include(LibFindMacros)

find_path(ftd2xx_INCLUDE_DIR
  NAMES ftd2xx.h
  PATHS "/usr/include"
)

find_library(ftd2xx_LIBRARY
  NAMES libftd2xx.so
  PATHS "/usr/lib" "/usr/local/lib"
)


set(ftd2xx_PROCESS_INCLUDES ftd2xx_INCLUDE_DIR)
set(ftd2xx_PROCESS_LIBS ftd2xx_LIBRARY)
libfind_process(ftd2xx)


PRINT_LIBRARY_STATUS(ftd2xx
  DETAILS "[${ftd2xx_LIBRARIES}][${ftd2xx_INCLUDE_DIRS}]"
)
