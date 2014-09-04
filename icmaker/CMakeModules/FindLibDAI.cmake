# - Try to find LibDAI
# Once done, this will define
#
#  LibDAI_FOUND - system has LibDAI
#  LibDAI_INCLUDE_DIRS - the LibDAI include directories
#  LibDAI_LIBRARIES - link these to use LibDAI

IF( LibDAI_FOUND )
   # in cache already
   SET( LibDAI_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(LibDAI_PKGCONF libdai)

# Include dir
find_path(LibDAI_INCLUDE_DIR
  NAMES dai/factorgraph.h dai/var.h
  PATHS ${LibDAI_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(LibDAI_LIBRARY
  NAMES dai
  PATHS ${LibDAI_PKGCONF_LIBRARY_DIRS}
)

find_library(gmpxx_LIBRARY
  NAMES gmpxx
  PATHS ${LibDAI_PKGCONF_LIBRARY_DIRS}
)

find_library(gmp_LIBRARY
  NAMES gmp
  PATHS ${LibDAI_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(LibDAI_PROCESS_INCLUDES LibDAI_INCLUDE_DIR)
set(LibDAI_PROCESS_LIBS LibDAI_LIBRARY gmpxx_LIBRARY gmp_LIBRARY)
libfind_process(LibDAI)

PRINT_LIBRARY_STATUS(LibDAI
  DETAILS "[${LibDAI_LIBRARIES}][${LibDAI_INCLUDE_DIRS}]"
)
