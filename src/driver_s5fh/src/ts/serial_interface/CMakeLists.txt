# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("ts_SVHSendPacket" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  SVHSendPacketTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${DRIVER_SVH_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  driver_svh
  icl_core
  icl_core_thread
  icl_comm
  )

ICMAKER_BUILD_TEST()

############################################################################

ICMAKER_SET("ts_SVHReceive" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  SVHReceiveTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${DRIVER_SVH_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  driver_svh
  icl_core
  icl_core_thread
  icl_comm
  )

ICMAKER_BUILD_TEST()

#############################################################################

ICMAKER_SET("ts_SVHSendFeedbackPacket" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  SVHSendFeedbackPacketTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${DRIVER_SVH_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  driver_svh
  icl_core
  icl_core_thread
  icl_comm
  )

ICMAKER_BUILD_TEST()

##############################################################################

ICMAKER_SET("ts_SVHReceiveFeedbackPacket" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  SVHReceiveFeedbackPacketTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${DRIVER_SVH_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  driver_svh
  icl_core
  icl_core_thread
  icl_comm
  )

ICMAKER_BUILD_TEST()

##############################################################################

ICMAKER_SET("ts_SVHControllerInitSendTest" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  SVHControllerInitSendTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${DRIVER_SVH_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  driver_svh
  icl_core
  icl_core_thread
  icl_comm
  )

ICMAKER_BUILD_TEST()

##############################################################################

ICMAKER_SET("ts_SVHFingerManagerTest" IDE_FOLDER ${DRIVER_SVH_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  SVHFingerManagerTest.cpp
  )

ICMAKER_INCLUDE_DIRECTORIES( ${DRIVER_SVH_INCLUDE_DIRS} )

ICMAKER_EXTERNAL_DEPENDENCIES(
  )

ICMAKER_INTERNAL_DEPENDENCIES(
  driver_svh
  icl_core
  icl_core_thread
  icl_comm
  )

ICMAKER_BUILD_TEST()
