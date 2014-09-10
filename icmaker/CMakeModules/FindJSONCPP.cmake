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

# - Try to find libjsoncpp
# Once done, this will define
#
# JSONCPP_FOUND - system has libjsoncpp
# JSONCPP_INCLUDE_DIRS - the libjsoncpp include directories
# JSONCPP_LIBRARIES - link these to use libjsoncpp

include(LibFindMacros)

IF (UNIX)
  libfind_pkg_check_modules(JSONCPP_PKGCONF jsoncpp)

  find_path(JSONCPP_INCLUDE_DIR
    NAMES json/json.h
    PATHS ${JSONCPP_ROOT}/include ${JSONCPP_PKGCONF_INCLUDE_DIRS}
    )

  find_library(JSONCPP_LIBRARY
    NAMES jsoncpp
    PATHS ${JSONCPP_ROOT}/lib ${JSONCPP_PKGCONF_LIBRARY_DIRS}
    )
ELSEIF (WIN32)
  find_path(JSONCPP_INCLUDE_DIR
    NAMES json/json.h
    PATHS ${JSONCPP_ROOT}/include ${CMAKE_INCLUDE_PATH}
    )
  find_library(JSONCPP_LIBRARY
    NAMES libjsoncpp
    PATHS ${JSONCPP_ROOT}/lib ${CMAKE_LIB_PATH}
    )
ENDIF()

set(JSONCPP_PROCESS_INCLUDES JSONCPP_INCLUDE_DIR JSONCPP_INCLUDE_DIRS)
set(JSONCPP_PROCESS_LIBS JSONCPP_LIBRARY JSONCPP_LIBRARIES)
libfind_process(JSONCPP)
