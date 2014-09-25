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

# - Try to find Libwebsockets
# Once done this will define
#  libwebsockets_FOUND - System has Libwebsockets
#  libwebsockets_INCLUDE_DIRS - The Libwebsockets include directories
#  libwebsockets_LIBRARIES - link these to use Libwebsockets

include(PrintLibraryStatus)
include(LibFindMacros)


find_path(libwebsockets_INCLUDE_DIR
  NAMES libwebsockets.h
  PATHS "/usr/share/include"
)

find_library(libwebsockets_LIBRARY
  NAMES libwebsockets.so
  PATHS "/usr/lib" "/usr/local/lib"
)


set(libwebsockets_PROCESS_INCLUDES libwebsockets_INCLUDE_DIR)
set(libwebsockets_PROCESS_LIBS libwebsockets_LIBRARY)
libfind_process(libwebsockets)


PRINT_LIBRARY_STATUS(libwebsockets
  DETAILS "[${libwebsockets_LIBRARIES}][${libwebsockets_INCLUDE_DIRS}]"
)

