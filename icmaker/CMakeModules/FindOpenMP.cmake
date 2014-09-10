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

include(${CMAKE_ROOT}/Modules/FindOpenMP.cmake)

IF(OPENMP_FOUND)
  SET(OpenMP_DEFINITIONS ${OpenMP_CXX_FLAGS})
  IF(NOT WIN32)
    SET(OpenMP_LIBRARIES ${OpenMP_CXX_FLAGS})
  ENDIF()
ENDIF()
