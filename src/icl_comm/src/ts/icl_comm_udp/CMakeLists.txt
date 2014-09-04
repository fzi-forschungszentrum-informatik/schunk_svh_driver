# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("ts_icl_comm_udp" IDE_FOLDER ${ICL_COMM_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  test_runner.cpp
  TestUdpImageSender.cpp
  )

ICMAKER_EXTERNAL_DEPENDENCIES(
  CppUnit
  OpenCV
  Qt_QtNetwork
)

ICMAKER_INTERNAL_DEPENDENCIES(
	icl_core
	icl_core_logging
	icl_comm_udp
	icl_comm_udp_image
  )

ICMAKER_BUILD_TEST()
