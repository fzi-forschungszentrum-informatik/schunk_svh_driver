# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("driver_svh" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  Logging.cpp
  SVHController.cpp
  SVHFeedbackPollingThread.cpp
  SVHFingerManager.cpp
  SVHReceiveThread.cpp
  SVHSerialInterface.cpp
  SVHSerialPacket.cpp
)

ICMAKER_ADD_HEADERS(
  ImportExport.h
  Logging.h
  SVHController.h
  SVHSerialPacket.h
  SVHPositionSettings.h
  SVHCurrentSettings.h
  SVHFirmwareInfo.h
  SVHControllerFeedback.h
  SVHControllerState.h
  SVHControlCommand.h
  SVHFeedbackPollingThread.h
  SVHFingerManager.h
  SVHReceiveThread.h
  SVHSerialInterface.h
  SVHEncoderSettings.h
  SVHHomeSettings.h
)

ICMAKER_LOCAL_CPPDEFINES(-DDRIVER_SVH_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_DRIVER_SVH_)
ICMAKER_INCLUDE_DIRECTORIES(${DRIVER_SVH_INCLUDE_DIRS})

#non exported deps
ICMAKER_DEPENDENCIES(
  icl_core
)

#Exported deps
ICMAKER_DEPENDENCIES(EXPORT
  icl_core
  icl_comm
  Boost_THREAD
)

# Websockets are a later expansion that is not considered at the moment but prepared for
ICMAKER_DEPENDENCIES(OPTIONAL EXPORT
 icl_comm_websocket
)


ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(${icmaker_target})

#----------------------------------------------------------------------
# Doxygen configuration
#----------------------------------------------------------------------

SET(IC_WORKSPACE_DOXYGEN_PROJECT_NAME "driver_svh" CACHE INTERNAL "")
SET(IC_WORKSPACE_DOXYGEN_CONFIG_OVERRIDES "FILE_PATTERNS = *.h *.hpp *.doxy *.cpp" CACHE INTERNAL "")
