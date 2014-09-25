# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# To be used by other modules
SET(ICL_COMM_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}/src" CACHE INTERNAL "")
SET(ICL_COMM_IDL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(ICL_COMM_IDE_FOLDER "icl_comm")

IF("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
  MESSAGE(STATUS "Package icl_comm has not been ported to Windows yet, ignoring.")
ELSE()
  ICMAKER_REGISTER_PACKAGE(icl_comm)

  ADD_SUBDIRECTORY (src/icl_comm)
  ADD_SUBDIRECTORY (src/icl_comm_serial)
  ADD_SUBDIRECTORY (src/icl_comm_websocket)

  ICMAKER_CONFIGURE_PACKAGE()

ENDIF()
