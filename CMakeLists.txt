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

SET(ICMAKER_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icmaker")
INCLUDE(${ICMAKER_DIRECTORY}/CMakeLists.txt)

ADD_SUBDIRECTORY (src/tinyxml)
ADD_SUBDIRECTORY (src/icl_core)
ADD_SUBDIRECTORY (src/icl_comm)
ADD_SUBDIRECTORY (src/driver_svh)

cmake_minimum_required(VERSION 2.8.3)
project(schunk_svh_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    rospy
    rqt_gui
    rqt_gui_py
    xacro
)

catkin_python_setup()

#dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/svh.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS roscpp icl_core icl_comm driver_svh
)


###########
## Build ##
###########

# mesh file generations
# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl


foreach(dir description/meshes description/urdf)
   install(DIRECTORY ${dir}/ 
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ICL_COMM_INCLUDE_DIRS}
  ${ICL_CORE_INCLUDE_DIRS}
  ${DRIVER_SVH_INCLUDE_DIRS}
)

# Additional compiler definitions
add_definitions(
  ${ICL_COMM_DEFINITIONS}
  ${ICL_CORE_DEFINITIONS}
)

# Declare a cpp executable
add_executable(svh_controller
  src/SVHNode.cpp
)

# Simple test programs
add_executable(svh_sin_test
  src/SVHSinTest.cpp
)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(svh_controller
  icl_comm
  icl_core
  driver_svh
  ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_controller
  ${catkin_LIBRARIES}
  ${ICL_COMM_LIBRARIES}
  ${ICL_CORE_LIBRARIES}
  ${DRIVER_SVH_LIBRARIES}
)


add_dependencies(svh_sin_test
  icl_comm
  icl_core
)

## Specify libraries to link a library or executable target against
target_link_libraries(svh_sin_test
  ${catkin_LIBRARIES}
  ${ICL_COMM_LIBRARIES}
  ${ICL_CORE_LIBRARIES}
)

#############
## Install ##
#############
# Special python handling
install(PROGRAMS
  src/rqt_svh_reset_gui/svh_reset_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install targets of libs and executables
install(TARGETS svh_controller svh_sin_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the headers
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#config files
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc)

#udev rules
install(DIRECTORY udev/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/udev)

# Individual files (Quick commands and the urdf.rviz)
install(FILES quick_commands plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install resources
install(DIRECTORY resource
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Helper for installing the udev rules in an easy fashion and without any conflicts
install(PROGRAMS udev/create_udev_rules
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
