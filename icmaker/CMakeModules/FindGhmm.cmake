# - Try to find Ghmm
# Once done, this will define
#
#  Ghmm_FOUND - system has Ghmm
#  Ghmm_INCLUDE_DIRS - the Ghmm include directories
#  Ghmm_LIBRARIES - link these to use Ghmm

IF(Ghmm_FOUND)
   # in cache already
   SET( Ghmm_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Ghmm_PKGCONF ghmm)

# Include dir
find_path(Ghmm_INCLUDE_DIR
  NAMES ghmm/ghmm.h
  PATHS ${Ghmm_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Ghmm_LIBRARY
  NAMES ghmm
  PATHS ${Ghmm_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Ghmm_PROCESS_INCLUDES Ghmm_INCLUDE_DIR)
set(Ghmm_PROCESS_LIBS Ghmm_LIBRARY)
libfind_process(Ghmm)

PRINT_LIBRARY_STATUS(Ghmm
  DETAILS "[${Ghmm_LIBRARIES}][${Ghmm_INCLUDE_DIRS}]"
)
