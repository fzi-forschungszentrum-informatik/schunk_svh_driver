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

# Extend the CMake FindThreads module.

include(${CMAKE_ROOT}/Modules/FindThreads.cmake)

IF (Threads_FOUND)
  IF (CMAKE_USE_PTHREADS_INIT)
    SET(Threads_DEFINITIONS -D_IC_BUILDER_PTHREAD_)
  ENDIF (CMAKE_USE_PTHREADS_INIT)
  SET(Threads_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
ENDIF (Threads_FOUND)
