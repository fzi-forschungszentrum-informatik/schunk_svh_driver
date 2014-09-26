# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# To be used by other modules
SET(DRIVER_SVH_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}/src" CACHE INTERNAL "")
SET(DRIVER_SVH_IDL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(DRIVER_SVH_IDE_FOLDER "driver_svh")

ICMAKER_REGISTER_PACKAGE(driver_svh)

ADD_SUBDIRECTORY (src/driver_svh)
ADD_SUBDIRECTORY (src/ts/driver_svh)
ADD_SUBDIRECTORY (src/ts/serial_interface)

ICMAKER_CONFIGURE_PACKAGE()
