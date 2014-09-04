# - Try to find Ardcom
# Once done, this will define
#
#  Ardcom_FOUND - system has Ardcom
#  Ardcom_INCLUDE_DIRS - the Ardcom include directories
#  Ardcom_LIBRARIES - link these to use Ardcom

IF( Ardcom_FOUND )
   # in cache already
   SET( Ardcom_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Ardcom_PKGCONF Ardcom)

# Include dir
find_path(Ardcom_INCLUDE_DIR
  NAMES ardcom/ardcom.h
  PATHS ${Ardcom_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Ardcom_LIBRARY
  NAMES libardcom.a libardcomsf.a
  PATHS ${Ardcom_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Ardcom_PROCESS_INCLUDES Ardcom_INCLUDE_DIR)
set(Ardcom_PROCESS_LIBS Ardcom_LIBRARY)
libfind_process(Ardcom)

PRINT_LIBRARY_STATUS(Ardcom
  DETAILS "[${Ardcom_LIBRARIES}][${Ardcom_INCLUDE_DIRS}]"
)
