# - Try to find OsmScoutMap
# Once done, this will define
#
#  OsmScoutMap_FOUND - system has OsmScoutMap
#  OsmScoutMap_INCLUDE_DIRS - the OsmScoutMap include directories
#  OsmScoutMap_LIBRARIES - link these to use OsmScoutMap

IF( OsmScoutMap_FOUND )
   # in cache already
   SET( OsmScoutMap_FIND_QUIETLY TRUE )
ENDIF()

INCLUDE(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(OsmScoutMap_PKGCONF libosmscout-map)

# Include dir
find_path(OsmScoutMap_INCLUDE_DIR
  NAMES osmscout/MapPainter.h
  PATHS ${OsmScoutMap_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(OsmScoutMap_LIBRARY
  NAMES osmscoutmap
  PATHS ${OsmScoutMap_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(OsmScoutMap_PROCESS_INCLUDES OsmScoutMap_INCLUDE_DIR)
set(OsmScoutMap_PROCESS_LIBS OsmScoutMap_LIBRARY)
libfind_process(OsmScoutMap)

PRINT_LIBRARY_STATUS(OsmScoutMap
  DETAILS "[${OsmScoutMap_LIBRARIES}][${OsmScoutMap_INCLUDE_DIRS}]"
)

