cmake_minimum_required(VERSION 2.8.3)
project(s5fh_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(icl_comm)
find_package(icl_core)
find_package(driver_s5fh)

#dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/s5fh.cfg
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
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS roscpp icl_core icl_comm driver_s5fh
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${icl_comm_INCLUDE_DIRS}
  ${icl_core_INCLUDE_DIRS}
  ${driver_s5fh_INCLUDE_DIRS}
)

# Additional compiler definitions
add_definitions(
  ${icl_comm_DEFINITIONS}
  ${icl_core_DEFINITIONS}
)

# Declare a cpp executable
add_executable(s5fh_controller
  src/s5fh_controller.cpp
)

# Add cmake target dependencies of the executable/library
# as an example, message headers may need to be generated before nodes
add_dependencies(s5fh_controller
  icl_comm
  icl_core
  driver_s5fh
  ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(s5fh_controller
  ${catkin_LIBRARIES}
  ${icl_comm_LIBRARIES}
  ${icl_core_LIBRARIES}
  ${driver_s5fh_LIBRARIES}
)

#############
## Install ##
#############

