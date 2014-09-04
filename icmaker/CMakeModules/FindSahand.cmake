# - Try to find SAHand
# Once done, this will define
#
#  Sahand_FOUND - system has Reflexx
#  Sahand_INCLUDE_DIRS - the Reflexx include directories
#  Sahand_LIBRARIES - link these to use Reflexx

IF( Sahand_FOUND )
   # in cache already
   SET( Sahand_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Sahand_PKGCONF sahand)

# Include dir
find_path(Sahand_INCLUDE_DIR
  NAMES sahand/SAHandCtrlApi.h
  PATHS ${Sahand_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Sahand_LIBRARY
  NAMES libsahand-static.a
  PATHS ${Sahand_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Sahand_PROCESS_INCLUDES Sahand_INCLUDE_DIR)
set(Sahand_PROCESS_LIBS Sahand_LIBRARY)
libfind_process(Sahand)

PRINT_LIBRARY_STATUS(Sahand
  DETAILS "[${Sahand_LIBRARIES}][${Sahand_INCLUDE_DIRS}]"
)
