# - Try to find FLIRTLib
# Once done, this will define
#
#  FLIRTLib_FOUND - system has FLIRTLib
#  FLIRTLib_INCLUDE_DIRS - the FLIRTLib include directories
#  FLIRTLib_LIBRARY_DIRS - the FLIRTLib library directories
#  FLIRTLib_LIBRARIES - link these to use FLIRTLib
#  FLIRTLib_LDFLAGS
#  FLIRTLib_LDFLAGS_OTHER
#  FLIRTLib_CFLAGS
#  FLIRTLib_CFLAGS_OTHER

IF (FLIRTLib_FOUND)
  # In cache already
  SET(FLIRTLib_FIND_QUIETLY TRUE)
ENDIF ()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(FLIRTLib_PKGCONF flirtlib)

# Include dir
find_path(FLIRTLib_INCLUDE_DIR
  NAMES flirtlib/feature/InterestPoint.h
  PATHS ${FLIRTLib_PKGCONF_INCLUDE_DIRS} "/opt/local/include"
)

# Finally the libraries themselves
set(_FLIRTLib_REQUIRED_LIBS
  flirt_sensors
  flirt_sensorstream
  flirt_geometry
  flirt_feature
  flirt_utils
  )
foreach (i ${_FLIRTLib_REQUIRED_LIBS})
  find_library(_FLIRTLib_LIBRARY
    NAMES ${i}
    PATHS ${FLIRTLib_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
    )
  SET(_FLIRTLib_LIBRARIES ${_FLIRTLib_LIBRARIES} ${_FLIRTLib_LIBRARY})
  UNSET(_FLIRTLib_LIBRARY)
  UNSET(_FLIRTLib_LIBRARY CACHE)
endforeach (i)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.

set(FLIRTLib_PROCESS_INCLUDES FLIRTLib_INCLUDE_DIR)
set(FLIRTLib_PROCESS_LIBS _FLIRTLib_LIBRARIES)
libfind_process(FLIRTLib)
UNSET(_FLIRTLib_LIBRARIES)
UNSET(_FLIRTLib_LIBRARIES CACHE)

If (FLIRTLib_FOUND)
  SET(FLIRTLib_LIBRARY_DIRS ${FLIRTLib_PKGCONF_LIBRARY_DIRS} CACHE INTERNAL "")
  SET(FLIRTLib_LDFLAGS ${FLIRTLib_PKGCONF_LDFLAGS} CACHE INTERNAL "")
  SET(FLIRTLib_LDFLAGS_OTHER ${FLIRTLib_PKGCONF_LDFLAGS_OTHER} CACHE INTERNAL "")
  SET(FLIRTLib_CFLAGS ${FLIRTLib_PKGCONF_CFLAGS} CACHE INTERNAL "")
  SET(FLIRTLib_CFLAGS_OTHER ${FLIRTLib_PKGCONF_CFLAGS_OTHER} CACHE INTERNAL "")
ENDIF (FLIRTLib_FOUND)

PRINT_LIBRARY_STATUS(FLIRTLib
  DETAILS "[${FLIRTLib_LIBRARY_DIRS}][${FLIRTLib_LIBRARIES}][${FLIRTLib_INCLUDE_DIRS}]"
  )
