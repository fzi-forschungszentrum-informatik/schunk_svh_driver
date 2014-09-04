# - Try to find CSMEigen
# Once done, this will define
#
#  CSMEigen_FOUND - system has CSMEigen
#  CSMEigen_INCLUDE_DIRS - the CSMEigen include directories
#  CSMEigen_LIBRARY_DIRS - the CSMEigen library directories
#  CSMEigen_LIBRARIES - link these to use CSMEigen

IF(CSMEigen_FOUND)
  # In cache already
  SET(CSMEigen_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

IF ((NOT DEFINED CSMEigen_ROOT OR CSMEigen_ROOT STREQUAL "") AND ("$ENV{CSMEigen_ROOT}" STREQUAL ""))
  # Use pkg-config to get hints about paths
  libfind_pkg_check_modules(CSMEigen_PKGCONF csm)
ELSE ()
  IF (CSMEigen_ROOT STREQUAL "")
    SET(CSMEigen_ROOT $ENV{CSMEigen_ROOT})
  ENDIF (CSMEigen_ROOT STREQUAL "")
  SET(CSMEigen_PKGCONF_INCLUDE_DIRS ${CSMEigen_ROOT}/include)
  SET(CSMEigen_PKGCONF_LIBRARY_DIRS ${CSMEigen_ROOT}/lib)
ENDIF ()

# Include dir
find_path(CSMEigen_INCLUDE_DIR
  NAMES csm/csm.h gsl_eigen/gsl_eigen.h
  PATHS ${CSMEigen_PKGCONF_INCLUDE_DIRS} "/opt/local/include"
)

# Finally the library itself
find_library(CSMEigen_LIBRARY
  NAMES csm_eigen
  PATHS ${CSMEigen_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(CSMEigen_PROCESS_INCLUDES CSMEigen_INCLUDE_DIR)
set(CSMEigen_PROCESS_LIBS CSMEigen_LIBRARY)
libfind_process(CSMEigen)

If (CSMEigen_FOUND)
  SET(CSMEigen_LIBRARY_DIRS ${CSMEigen_PKGCONF_LIBRARY_DIRS} CACHE INTERNAL "")
  SET(CSMEigen_LDFLAGS ${CSMEigen_PKGCONF_LDFLAGS} CACHE INTERNAL "")
  SET(CSMEigen_LDFLAGS_OTHER ${CSMEigen_PKGCONF_LDFLAGS_OTHER} CACHE INTERNAL "")
  SET(CSMEigen_CFLAGS ${CSMEigen_PKGCONF_CFLAGS} CACHE INTERNAL "")
  SET(CSMEigen_CFLAGS_OTHER ${CSMEigen_PKGCONF_CFLAGS_OTHER} CACHE INTERNAL "")
ENDIF (CSMEigen_FOUND)

PRINT_LIBRARY_STATUS(CSMEigen
  DETAILS "[${CSMEigen_LIBRARY_DIRS}][${CSMEigen_LIBRARIES}][${CSMEigen_INCLUDE_DIRS}]"
  )
