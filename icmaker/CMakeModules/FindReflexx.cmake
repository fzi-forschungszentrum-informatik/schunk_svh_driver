# - Try to find Reflexx
# Once done, this will define
#
#  Reflexx_FOUND - system has Reflexx
#  Reflexx_INCLUDE_DIRS - the Reflexx include directories
#  Reflexx_LIBRARIES - link these to use Reflexx

IF( Reflexx_FOUND )
   # in cache already
   SET( Reflexx_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Reflexx_PKGCONF reflexx)

# Include dir
find_path(Reflexx_INCLUDE_DIR
  NAMES reflexxes/ReflexxesAPI.h
  PATHS ${Reflexx_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Reflexx_LIBRARY
  NAMES ReflexxesTypeIV
  PATHS ${Reflexx_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Reflexx_PROCESS_INCLUDES Reflexx_INCLUDE_DIR)
set(Reflexx_PROCESS_LIBS Reflexx_LIBRARY)
libfind_process(Reflexx)

PRINT_LIBRARY_STATUS(Reflexx
  DETAILS "[${Reflexx_LIBRARIES}][${Reflexx_INCLUDE_DIRS}]"
)
