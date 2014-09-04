# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("icl_comm_websocket" IDE_FOLDER ${ICL_COMM_IDE_FOLDER})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

FIND_PACKAGE(JSONCPP)

ICMAKER_ADD_SOURCES(
 UnixSocket.cpp
 RobotState.cpp
 LWA4PState.cpp
 SVHState.cpp
 WsBroadcaster.cpp
)

ICMAKER_ADD_HEADERS(
  UnixSocket.h
  WsBroadcaster.h
  RobotState.h
  LWA4PState.h
  SVHState.h
)


#SET(private_headers
# Logging.h
#  )

ICMAKER_INCLUDE_DIRECTORIES(${ICL_COMM_INCLUDE_DIRS})
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_ICL_COMM_WEBSOCKET_)

ICMAKER_DEPENDENCIES( EXPORT
  icl_core
  icl_core_logging
  jsoncpp
)

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(${icmaker_target})


###################################
ICMAKER_SET("ts_WsBroadcaster" IDE_FOLDER ${ICL_COMM_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  WsBroadcasterTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${ICL_COMM_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  icl_core
  icl_comm_websocket
  )

ICMAKER_BUILD_PROGRAM()
