# - Try to find LibUSB
# Once done, this will define
#
#  LibUSB_FOUND - system has LibUSB
#  LibUSB_INCLUDE_DIRS - the LibUSB include directories
#  LibUSB_LIBRARIES - link these to use LibUSB

IF( LibUSB_FOUND )
   # in cache already
   SET( LibUSB_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(LibUSB_PKGCONF ltdl)

# Include dir
find_path(LibUSB_INCLUDE_DIR
  NAMES usb.h
  PATHS ${LibUSB_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(LibUSB_LIBRARY
  NAMES usb
  PATHS ${LibUSB_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(LibUSB_PROCESS_INCLUDES LibUSB_INCLUDE_DIR)
set(LibUSB_PROCESS_LIBS LibUSB_LIBRARY)
libfind_process(LibUSB)

PRINT_LIBRARY_STATUS(LibUSB
  DETAILS "[${LibUSB_LIBRARIES}][${LibUSB_INCLUDE_DIRS}]"
)

