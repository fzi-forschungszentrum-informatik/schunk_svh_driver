# - Try to find LibXDO
# Once done, this will define
#
#  XDO_FOUND - system has LibXDO
#  XDO_INCLUDE_DIR - the LibXDO include directories
#  XDO_LIBRARY - link these to use LibXDO

if ( XDO_FOUND )
   # in cache already
   SET( XDO_FIND_QUIETLY TRUE )
endif ()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(XDO_PKGCONF xdo)

# Include dir
find_path(XDO_INCLUDE_DIR
  NAMES xdo.h
  PATHS ${XDO_PKGCONF_INCLUDE_DIRS} "/usr/include/"
)

# Finally the library itself
find_library(XDO_LIBRARY
  NAMES xdo
  PATHS ${XDO_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(XDO_PROCESS_INCLUDES XDO_INCLUDE_DIR)
set(XDO_PROCESS_LIBS XDO_LIBRARY)
libfind_process(XDO)

PRINT_LIBRARY_STATUS(XDO
  DETAILS "[${XDO_LIBRARIES}][${XDO_INCLUDE_DIRS}]"
)
