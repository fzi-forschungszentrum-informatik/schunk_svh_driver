# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("icl_comm_rosbridge" IDE_FOLDER ${ICL_COMM_IDE_FOLDER})

find_package(JSONCPP)

ICMAKER_ADD_SOURCES(
  Logging.cpp
  RosbridgeIoStream.cpp
  RosbridgeClient.cpp
)

ICMAKER_ADD_HEADERS(
  RosbridgeIoStream.h
  RosbridgeClient.h
)

SET(private_headers
  Logging.h
  )

ICMAKER_INCLUDE_DIRECTORIES(${ICL_COMM_INCLUDE_DIRS})

ICMAKER_INTERNAL_DEPENDENCIES(
  jsoncpp
  icl_core
  icl_core_logging
  icl_comm_tcp
)

ICMAKER_BUILD_LIBRARY()
