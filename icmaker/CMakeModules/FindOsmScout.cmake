# - Try to find OsmScout
# Once done, this will define
#
#  OsmScout_FOUND - system has OsmScout
#  OsmScout_INCLUDE_DIRS - the OsmScout include directories
#  OsmScout_LIBRARIES - link these to use OsmScout

IF( OsmScout_FOUND )
   # in cache already
   SET( OsmScout_FIND_QUIETLY TRUE )
ENDIF()

INCLUDE(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(OsmScout_PKGCONF libosmscout)

# Include dir
find_path(OsmScout_INCLUDE_DIR
  NAMES osmscout/Tag.h
  PATHS ${OsmScout_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(OsmScout_LIBRARY
  NAMES osmscout
  PATHS ${OsmScout_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(OsmScout_PROCESS_INCLUDES OsmScout_INCLUDE_DIR)
set(OsmScout_PROCESS_LIBS OsmScout_LIBRARY)
libfind_process(OsmScout)

PRINT_LIBRARY_STATUS(OsmScout
  DETAILS "[${OsmScout_LIBRARIES}][${OsmScout_INCLUDE_DIRS}]"
)

