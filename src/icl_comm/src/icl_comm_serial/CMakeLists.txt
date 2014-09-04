# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("icl_comm_serial" IDE_FOLDER ${ICL_COMM_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  Logging.cpp
  Serial.cpp
  SerialFlags.cpp
)

ICMAKER_ADD_HEADERS(
  ImportExport.h
  Logging.h
  Serial.h
  SerialFlags.h
)

ICMAKER_LOCAL_CPPDEFINES(-DICL_COMM_SERIAL_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_ICL_COMM_SERIAL_)
ICMAKER_INCLUDE_DIRECTORIES(${ICL_COMM_INCLUDE_DIRS})

ICMAKER_INTERNAL_DEPENDENCIES(
  icl_core
  icl_core_logging
)

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(${icmaker_target})
