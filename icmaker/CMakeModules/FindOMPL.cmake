# - Try to find OMPL
# Once done, this will define
#
#  OMPL_FOUND - system has OMPL
#  OMPL_INCLUDE_DIRS - the OMPL include directories
#  OMPL_LIBRARIES - link these to use OMPL

IF( OMPL_FOUND )
   # in cache already
   SET( OMPL_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(OMPL_PKGCONF ompl)

# Include dir
find_path(OMPL_INCLUDE_DIR
  NAMES ompl/base/StateSampler.h
  PATHS ${OMPL_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(OMPL_LIBRARY
  NAMES ompl
  PATHS ${OMPL_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
  NO_DEFAULT_PATH
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this lib depends on.
set(OMPL_PROCESS_INCLUDES OMPL_INCLUDE_DIR)
set(OMPL_PROCESS_LIBS OMPL_LIBRARY)
libfind_process(OMPL)

PRINT_LIBRARY_STATUS(OMPL
  DETAILS "[${OMPL_LIBRARIES}][${OMPL_INCLUDE_DIRS}]"
)
