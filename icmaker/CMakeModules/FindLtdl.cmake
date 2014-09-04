# - Try to find Ltdl
# Once done, this will define
#
#  Ltdl_FOUND - system has Ltdl
#  Ltdl_INCLUDE_DIR - the Ltdl include directories
#  Ltdl_LIBRARY - link these to use Ltdl

if ( Ltdl_FOUND )
   # in cache already
   SET( Ltdl_FIND_QUIETLY TRUE )
endif ()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
# libfind_pkg_check_modules(Ltdl_PKGCONF ltdl)

# Include dir
find_path(Ltdl_INCLUDE_DIR
  NAMES ltdl.h
  PATHS ${Ltdl_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Ltdl_LIBRARY
  NAMES ltdl
  PATHS ${Ltdl_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Ltdl_PROCESS_INCLUDES Ltdl_INCLUDE_DIR)
set(Ltdl_PROCESS_LIBS Ltdl_LIBRARY)
libfind_process(Ltdl)

PRINT_LIBRARY_STATUS(Ltdl
  DETAILS "[${Ltdl_LIBRARIES}][${Ltdl_INCLUDE_DIRS}]"
)
