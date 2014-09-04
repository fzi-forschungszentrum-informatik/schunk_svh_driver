# - Try to find Spacenav
# Once done, this will define
#
#  Spacenav_FOUND - system has Spacenav
#  Spacenav_INCLUDE_DIRS - the Spacenav include directories
#  Spacenav_LIBRARIES - link these to use Spacenav

IF( Spacenav_FOUND )
   # in cache already
   SET( Spacenav_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Spacenav_PKGCONF spnav)

# Include dir
find_path(Spacenav_INCLUDE_DIR
  NAMES spnav.h
  PATHS ${Spacenav_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Spacenav_LIBRARY
  NAMES spnav
  PATHS ${Spacenav_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Spacenav_PROCESS_INCLUDES Spacenav_INCLUDE_DIR)
set(Spacenav_PROCESS_LIBS Spacenav_LIBRARY)
libfind_process(Spacenav)

PRINT_LIBRARY_STATUS(Spacenav
  DETAILS "[${Spacenav_LIBRARIES}][${Spacenav_INCLUDE_DIRS}]"
)


