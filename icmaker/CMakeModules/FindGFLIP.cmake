# - Try to find GFLIP
# Once done, this will define
#
#  GFLIP_FOUND - system has GFLIP
#  GFLIP_INCLUDE_DIRS - the GFLIP include directories
#  GFLIP_LIBRARY_DIRS - the GFLIP library directories
#  GFLIP_LIBRARIES - link these to use GFLIP
#  GFLIP_LDFLAGS
#  GFLIP_LDFLAGS_OTHER
#  GFLIP_CFLAGS
#  GFLIP_CFLAGS_OTHER

IF (GFLIP_FOUND)
  # In cache already
  SET(GFLIP_FIND_QUIETLY TRUE)
ENDIF ()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(GFLIP_PKGCONF gflip)

# Include dir
find_path(GFLIP_INCLUDE_DIR
  NAMES gflip/gflip_engine.hpp
  PATHS ${GFLIP_PKGCONF_INCLUDE_DIRS} "/opt/local/include"
)

# Finally the libraries themselves
set(_GFLIP_REQUIRED_LIBS
  gflip
  gflip_vocabulary
  )
foreach (i ${_GFLIP_REQUIRED_LIBS})
  find_library(_GFLIP_LIBRARY
    NAMES ${i}
    PATHS ${GFLIP_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
    )
  SET(_GFLIP_LIBRARIES ${_GFLIP_LIBRARIES} ${_GFLIP_LIBRARY})
  UNSET(_GFLIP_LIBRARY)
  UNSET(_GFLIP_LIBRARY CACHE)
endforeach (i)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(GFLIP_PROCESS_INCLUDES GFLIP_INCLUDE_DIR)
set(GFLIP_PROCESS_LIBS _GFLIP_LIBRARIES)
libfind_process(GFLIP)

If (GFLIP_FOUND)
  SET(GFLIP_LIBRARY_DIRS ${GFLIP_PKGCONF_LIBRARY_DIRS} CACHE INTERNAL "")
  SET(GFLIP_LDFLAGS ${GFLIP_PKGCONF_LDFLAGS} CACHE INTERNAL "")
  SET(GFLIP_LDFLAGS_OTHER ${GFLIP_PKGCONF_LDFLAGS_OTHER} CACHE INTERNAL "")
  SET(GFLIP_CFLAGS ${GFLIP_PKGCONF_CFLAGS} CACHE INTERNAL "")
  SET(GFLIP_CFLAGS_OTHER ${GFLIP_PKGCONF_CFLAGS_OTHER} CACHE INTERNAL "")
ENDIF (GFLIP_FOUND)

PRINT_LIBRARY_STATUS(GFLIP
  DETAILS "[${GFLIP_LIBRARY_DIRS}][${GFLIP_LIBRARIES}][${GFLIP_INCLUDE_DIRS}]"
  )
