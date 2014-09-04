# - Try to find libarchive
#   (http://people.freebsd.org/~kientzle/libarchive/)
#
# Once done, this will define
#
#  LibArchive_FOUND - system has libarchive
#  LibArchive_INCLUDE_DIRS - the libarchive include directories
#  LibArchive_LIBRARIES - link these to use libarchive

IF( LibArchive_FOUND )
   # in cache already
   SET( LibArchive_FIND_QUIETLY TRUE )
ENDIF()

INCLUDE(LibFindMacros)
INCLUDE(PrintLibraryStatus)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(LibArchive_PKGCONF libarchive)

# Include dir
find_path(LibArchive_INCLUDE_DIR
  NAMES archive.h
  PATHS ${LibArchive_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(LibArchive_LIBRARY
  NAMES archive
  PATHS ${LibArchive_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(LibArchive_PROCESS_INCLUDES LibArchive_INCLUDE_DIR)
set(LibArchive_PROCESS_LIBS LibArchive_LIBRARY)
libfind_process(LibArchive)

PRINT_LIBRARY_STATUS(LibArchive
  DETAILS "[${LibArchive_LIBRARIES}][${LibArchive_INCLUDE_DIRS}]"
)

