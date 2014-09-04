# - Try to find CSM
# Once done, this will define
#
#  CSM_FOUND - system has CSM
#  CSM_INCLUDE_DIRS - the CSM include directories
#  CSM_LIBRARY_DIRS - the CSM library directories
#  CSM_LIBRARIES - link these to use CSM

IF(CSM_FOUND)
  # In cache already
  SET(CSM_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

IF ((NOT DEFINED CSM_ROOT OR CSM_ROOT STREQUAL "") AND ("$ENV{CSM_ROOT}" STREQUAL ""))
  # Use pkg-config to get hints about paths
  libfind_pkg_check_modules(CSM_PKGCONF csm)
ELSE ()
  IF (CSM_ROOT STREQUAL "")
    SET(CSM_ROOT $ENV{CSM_ROOT})
  ENDIF (CSM_ROOT STREQUAL "")
  SET(CSM_PKGCONF_INCLUDE_DIRS ${CSM_ROOT}/include)
  SET(CSM_PKGCONF_LIBRARY_DIRS ${CSM_ROOT}/lib)
ENDIF ()

# Include dir
find_path(CSM_INCLUDE_DIR
  NAMES csm/csm.h
  PATHS ${CSM_PKGCONF_INCLUDE_DIRS} "/opt/local/include"
)

# Finally the library itself
find_library(CSM_LIBRARY
  NAMES csm
  PATHS ${CSM_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(CSM_PROCESS_INCLUDES CSM_INCLUDE_DIR)
set(CSM_PROCESS_LIBS CSM_LIBRARY)
libfind_process(CSM)

If (CSM_FOUND)
  SET(CSM_LIBRARY_DIRS ${CSM_PKGCONF_LIBRARY_DIRS} CACHE INTERNAL "")
  SET(CSM_LDFLAGS ${CSM_PKGCONF_LDFLAGS} CACHE INTERNAL "")
  SET(CSM_LDFLAGS_OTHER ${CSM_PKGCONF_LDFLAGS_OTHER} CACHE INTERNAL "")
  SET(CSM_CFLAGS ${CSM_PKGCONF_CFLAGS} CACHE INTERNAL "")
  SET(CSM_CFLAGS_OTHER ${CSM_PKGCONF_CFLAGS_OTHER} CACHE INTERNAL "")
ENDIF (CSM_FOUND)

PRINT_LIBRARY_STATUS(CSM
  DETAILS "[${CSM_LIBRARY_DIRS}][${CSM_LIBRARIES}][${CSM_INCLUDE_DIRS}]"
  )
