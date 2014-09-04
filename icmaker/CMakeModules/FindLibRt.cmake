# - Try to find LibRt
# Once done, this will define
#
#  LibRt_FOUND - system has LibRt
#  LibRt_LIBRARIES - link these to use LibRt

IF( LibRt_FOUND )
   # in cache already
   SET( LibRt_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(LibRt_PKGCONF LibRt)

# Finally the library itself
find_library(LibRt_LIBRARY
  NAMES rt
  PATHS ${LibRt_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(LibRt_PROCESS_LIBS LibRt_LIBRARY)
libfind_process(LibRt)

PRINT_LIBRARY_STATUS(LibRt
  DETAILS "[${LibRt_LIBRARIES}]"
)
