# - Try to find GPSD
# Once done, this will define
#
#  GPSD_FOUND - system has GPSD
#  GPSD_INCLUDE_DIRS - the GPSD include directories
#  GPSD_LIBRARIES - link these to use GPSD

IF( GPSD_FOUND )
   # in cache already
   SET( GPSD_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(GPSD_PKGCONF libgps)

# Include dir
find_path(GPSD_INCLUDE_DIR
  NAMES gps.h
  PATHS ${GPSD_PKGCONF_INCLUDE_DIRS} "/opt/local/include"
)

# Finally the library itself
find_library(GPSD_LIBRARY
  NAMES gps
  PATHS ${GPSD_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(GPSD_PROCESS_INCLUDES GPSD_INCLUDE_DIR)
set(GPSD_PROCESS_LIBS GPSD_LIBRARY)
libfind_process(GPSD)

PRINT_LIBRARY_STATUS(GPSD
  DETAILS "[${GPSD_LIBRARIES}][${GPSD_INCLUDE_DIRS}]"
)

