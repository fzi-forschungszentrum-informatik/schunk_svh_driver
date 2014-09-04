# - Try to find Matio
# Once done, this will define
#
#  Matio_FOUND - system has Matio
#  Matio_INCLUDE_DIRS - the Matio include directories
#  Matio_LIBRARIES - link these to use Matio

IF(Matio_FOUND)
   # in cache already
   SET( Matio_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

if (MATIO_ROOT STREQUAL "" AND "$ENV{MATIO_ROOT}" STREQUAL "")
  # Use pkg-config to get hints about paths
  libfind_pkg_check_modules(Matio_PKGCONF matio)
else (MATIO_ROOT STREQUAL "" AND "$ENV{MATIO_ROOT}" STREQUAL "")
  if (MATIO_ROOT STREQUAL "")
    set(MATIO_ROOT $ENV{MATIO_ROOT})
  endif (MATIO_ROOT STREQUAL "")
  set(Matio_PKGCONF_INCLUDE_DIRS ${MATIO_ROOT}/include)
  set(Matio_PKGCONF_LIBRARY_DIRS ${MATIO_ROOT}/lib)
endif (MATIO_ROOT STREQUAL "" AND "$ENV{MATIO_ROOT}" STREQUAL "")

# Include dir
find_path(Matio_INCLUDE_DIR
  NAMES matio.h
  PATHS ${Matio_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Matio_LIBRARY
  NAMES matio
  PATHS ${Matio_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Matio_PROCESS_INCLUDES Matio_INCLUDE_DIR)
set(Matio_PROCESS_LIBS Matio_LIBRARY)
libfind_process(Matio)

PRINT_LIBRARY_STATUS(Matio
  DETAILS "[${Matio_LIBRARIES}][${Matio_INCLUDE_DIRS}]"
)
