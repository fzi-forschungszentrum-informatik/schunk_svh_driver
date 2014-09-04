# - Try to find Ncomrx
# Once done, this will define
#
#  Ncomrx_FOUND - system has Ncomrx
#  Ncomrx_INCLUDE_DIRS - the Ncomrx include directories
#  Ncomrx_LIBRARIES - link these to use Ncomrx

IF( Ncomrx_FOUND )
   # in cache already
   SET( Ncomrx_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Ncomrx_PKGCONF ncomrx)

# Include dir
find_path(Ncomrx_INCLUDE_DIR
  NAMES NComRxC.h
  PATHS ${Ncomrx_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Ncomrx_LIBRARY
  NAMES ncomrx
  PATHS ${Ncomrx_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Ncomrx_PROCESS_INCLUDES Ncomrx_INCLUDE_DIR)
set(Ncomrx_PROCESS_LIBS Ncomrx_LIBRARY)
libfind_process(Ncomrx)

PRINT_LIBRARY_STATUS(Ncomrx
  DETAILS "[${Ncomrx_LIBRARIES}][${Ncomrx_INCLUDE_DIRS}]"
)
