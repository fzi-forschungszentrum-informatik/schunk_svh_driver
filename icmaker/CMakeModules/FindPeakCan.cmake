# - Try to find PeakCan
# Once done, this will define
#
#  PeakCan_FOUND - system has PeakCan
#  PeakCan_INCLUDE_DIRS - the PeakCan include directories
#  PeakCan_LIBRARIES - link these to use PeakCan

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(PeakCan_PKGCONF libpcan)

# Include dir
find_path(PeakCan_INCLUDE_DIR
  NAMES libpcan.h
  PATHS ${PeakCan_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(PeakCan_LIBRARY
  NAMES pcan
  PATHS ${PeakCan_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(PeakCan_PROCESS_INCLUDES PeakCan_INCLUDE_DIR)
set(PeakCan_PROCESS_LIBS PeakCan_LIBRARY)
libfind_process(PeakCan)

if(PeakCan_FOUND)
  set(PeakCan_DEFINITIONS "-D_IC_BUILDER_CAN_PEAK_")
endif()
