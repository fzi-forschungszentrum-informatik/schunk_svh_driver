# - Try to find DC1394V2
# Once done, this will define
#
#  DC1394V2_FOUND - system has DC1394
#  DC1394V2_INCLUDE_DIRS - the DC1394 include directories
#  DC1394V2_LIBRARIES - link these to use DC1394

IF(DC1394V2_FOUND)
   # in cache already
   SET(DC1394V2_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(DC1394V2_PKGCONF libdc1394-2)

# Include dir
find_path(DC1394V2_INCLUDE_DIR
  NAMES dc1394/control.h
  PATHS ${DC1394V2_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(DC1394V2_LIBRARY
  NAMES dc1394
  PATHS ${DC1394V2_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(DC1394V2_PROCESS_INCLUDES DC1394V2_INCLUDE_DIR)
set(DC1394V2_PROCESS_LIBS DC1394V2_LIBRARY)
libfind_process(DC1394V2)

PRINT_LIBRARY_STATUS(DC1394V2
  DETAILS "[${DC1394V2_LIBRARIES}][${DC1394V2_INCLUDE_DIRS}]"
)
