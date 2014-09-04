# - Try to find Iconv
# Once done, this will define
#
#  Iconv_FOUND - system has Iconv
#  Iconv_INCLUDE_DIR - the Ltdl include directories
#  Iconv_LIBRARY - link these to use Ltdl

IF(Iconv_FOUND)
  # in cache already
  SET(Iconv_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

IF(ICONV_ROOT STREQUAL "" AND "$ENV{ICONV_ROOT}" STREQUAL "")
  # Use pkg-config to get hints about paths
  libfind_pkg_check_modules(Iconv_PKGCONF iconv)
ELSE()
  IF(ICONV_ROOT STREQUAL "")
    SET(ICONV_ROOT $ENV{ICONV_ROOT})
  ENDIF()
  SET(Iconv_PKGCONF_INCLUDE_DIRS ${ICONV_ROOT}/include)
  SET(Iconv_PKGCONF_LIBRARY_DIRS ${ICONV_ROOT}/lib)
ENDIF()

# Include dir
find_path(Iconv_INCLUDE_DIR
  NAMES iconv.h
  PATHS ${Iconv_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Iconv_LIBRARY
  NAMES iconv
  PATHS ${Iconv_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Iconv_PROCESS_INCLUDES Iconv_INCLUDE_DIR)
set(Iconv_PROCESS_LIBS Iconv_LIBRARY)
libfind_process(Iconv)

PRINT_LIBRARY_STATUS(Iconv
  DETAILS "[${Iconv_LIBRARIES}][${Iconv_INCLUDE_DIRS}]"
)
