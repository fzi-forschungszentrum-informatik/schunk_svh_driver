IF(Pthread_FOUND)
   # in cache already
   SET(Pthread_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Pthread_PKGCONF libpthread)

# Include dir
find_path(Pthread_INCLUDE_DIR
  NAMES pthread.h
  PATHS ${Pthread_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Pthread_LIBRARY
  NAMES pthread
  PATHS ${Pthread_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Pthread_PROCESS_INCLUDES Pthread_INCLUDE_DIR)
set(Pthread_PROCESS_LIBS Pthread_LIBRARY)
libfind_process(Pthread)

PRINT_LIBRARY_STATUS(Pthread
  DETAILS "[${Pthread_LIBRARIES}][${Pthread_INCLUDE_DIRS}]"
)
