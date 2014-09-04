IF(DL_FOUND)
   # in cache already
   SET(DL_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(DL_PKGCONF libdl)

# Include dir
find_path(DL_INCLUDE_DIR
  NAMES dlfcn.h
  PATHS ${DL_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(DL_LIBRARY
  NAMES dl
  PATHS ${DL_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(DL_PROCESS_INCLUDES DL_INCLUDE_DIR)
set(DL_PROCESS_LIBS DL_LIBRARY)
libfind_process(DL)

PRINT_LIBRARY_STATUS(DL
  DETAILS "[${DL_LIBRARIES}][${DL_INCLUDE_DIRS}]"
) 
