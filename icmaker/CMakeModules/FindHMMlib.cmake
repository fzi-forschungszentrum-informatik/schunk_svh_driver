# - Try to find HMMlib
# Once done, this will define
#
#  HMMlib_FOUND - system has HMMlib
#  HMMlib_INCLUDE_DIRS - the HMMlib include directories
#  HMMlib_CFLAGS - the HMMlib extra cflags (required for compilation)

IF(HMMlib_FOUND)
   # in cache already
   SET( HMMlib_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(HMMlib_PKGCONF hmmlib)

# Include dir
find_path(HMMlib_INCLUDE_DIR
  NAMES HMMlib/hmm.hpp
  PATHS ${HMMlib_PKGCONF_INCLUDE_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(HMMlib_PROCESS_INCLUDES HMMlib_INCLUDE_DIR)
libfind_process(HMMlib)
IF(HMMlib_FOUND)
  SET(HMMlib_CFLAGS -msse4 CACHE INTERNAL "")
ENDIF()

PRINT_LIBRARY_STATUS(HMMlib
  DETAILS "[${HMMlib_INCLUDE_DIRS}]"
)
