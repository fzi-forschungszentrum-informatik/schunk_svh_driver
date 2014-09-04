# - Try to find Cantools
# Once done, this will define
#
#  Cantools_FOUND - system has Cantools
#  Cantools_INCLUDE_DIRS - the Cantools include directories
#  Cantools_LIBRARIES - link these to use Cantools

IF( Cantools_FOUND )
   # in cache already
   SET( Cantools_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Cantools_PKGCONF cantools)

# Include dir
find_path(Cantools_INCLUDE_DIR
  NAMES dbcModel.h
  PATHS ${Cantools_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Cantools_LIBRARY
  NAMES candbc
  PATHS ${Cantools_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Cantools_PROCESS_INCLUDES Cantools_INCLUDE_DIR)
set(Cantools_PROCESS_LIBS Cantools_LIBRARY)
libfind_process(Cantools)

PRINT_LIBRARY_STATUS(Cantools
  DETAILS "[${Cantools_LIBRARIES}][${Cantools_INCLUDE_DIRS}]"
)
