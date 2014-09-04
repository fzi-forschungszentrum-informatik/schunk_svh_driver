# - Try to find HCRF
# Once done, this will define
#
#  HCRF_FOUND - system has HCRF
#  HCRF_INCLUDE_DIRS - the HCRF include directories
#  HCRF_LIBRARIES - link these to use HCRF

IF(HCRF_FOUND)
   # in cache already
   SET( HCRF_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(HCRF_PKGCONF hcrf)

# Include dir
find_path(HCRF_INCLUDE_DIR
  NAMES hCRF/hCRF.h
  PATHS ${HCRF_PKGCONF_INCLUDE_DIRS}
)

SET(libraries cgDescent hCRF lbfgs uncoptim)
FOREACH(library ${libraries})
  find_library(HCRF_LIBRARY_${library}
    NAMES ${library}
    PATHS ${HCRF_PKGCONF_LIBRARY_DIRS}
  )
  LIST(APPEND HCRF_LIBRARIES ${HCRF_LIBRARY_${library}})
ENDFOREACH()

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(HCRF_PROCESS_INCLUDES HCRF_INCLUDE_DIR)
set(HCRF_PROCESS_LIBS HCRF_LIBRARIES)
libfind_process(HCRF)

PRINT_LIBRARY_STATUS(HCRF
  DETAILS "[${HCRF_LIBRARIES}][${HCRF_INCLUDE_DIRS}]"
)
