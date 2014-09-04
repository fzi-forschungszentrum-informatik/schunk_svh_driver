# - Try to find OsmPBF
# Once done, this will define
#
#  OsmPBF_FOUND - system has OsmPBF
#  OsmPBF_INCLUDE_DIRS - the OsmPBF include directories
#  OsmPBF_LIBRARIES - link these to use OsmPBF

IF( OsmPBF_FOUND )
   # in cache already
   SET( OsmPBF_FIND_QUIETLY TRUE )
ENDIF()

INCLUDE(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(OsmPBF_PKGCONF libosmpbf)

# Include dir
find_path(OsmPBF_INCLUDE_DIR
  NAMES osmpbf/osmpbf.h
  PATHS ${OsmPBF_PKGCONF_INCLUDE_DIRS} "/usr/include/"
)

# Finally the library itself
find_library(OsmPBF_LIBRARY
  NAMES libosmpbf.a
  PATHS ${OsmPBF_PKGCONF_LIBRARY_DIRS} "/usr/lib/"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(OsmPBF_PROCESS_INCLUDES OsmPBF_INCLUDE_DIR)
set(OsmPBF_PROCESS_LIBS OsmPBF_LIBRARY)
libfind_process(OsmPBF)

PRINT_LIBRARY_STATUS(OsmPBF
  DETAILS "[${OsmPBF_LIBRARIES}][${OsmPBF_INCLUDE_DIRS}]"
)

