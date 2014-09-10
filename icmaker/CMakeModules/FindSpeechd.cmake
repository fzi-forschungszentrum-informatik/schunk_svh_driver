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

# - Try to find libspeechd-dev
# Once done this will define
#  Speechd_FOUND - System has libspeechd
#  Speechd_INCLUDE_DIRS - The libspeechd include directories
#  Speechd_LIBRARIES - link these to use libspeechd

include(PrintLibraryStatus)
include(LibFindMacros)


find_path(speechd_INCLUDE_DIR
  NAMES libspeechd.h
  PATHS "/usr/include"
)

find_library(speechd_LIBRARY
  NAMES libspeechd.so
  PATHS "/usr/lib" "/usr/local/lib"
)


set(speechd_PROCESS_INCLUDES speechd_INCLUDE_DIR)
set(speechd_PROCESS_LIBS speechd_LIBRARY)
libfind_process(speechd)


PRINT_LIBRARY_STATUS(speechd
  DETAILS "[${speechd_LIBRARIES}][${speechd_INCLUDE_DIRS}]"
)

