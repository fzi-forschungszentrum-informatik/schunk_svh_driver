# - Try to find DXFLib
# Once done, this will define
#
#  DXFLib_FOUND - system has DXFLib
#  DXFLib_INCLUDE_DIRS - the DXFLib include directories
#  DXFLib_LIBRARIES - link these to use DXFLib

IF (DXFLib_FOUND)
  # in cache already
  SET(DXFLib_FIND_QUIETLY TRUE)
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(DXFLib_PKGCONF dxflib)

# Include dir
find_path(DXFLib_INCLUDE_DIR
  NAMES dl_dxf.h
  PATHS ${DXFLib_PKGCONF_INCLUDE_DIRS}
  )

# Finally the library itself
find_library(DXFLib_LIBRARY
  NAMES dxflib
  PATHS ${DXFLib_PKGCONF_LIBRARY_DIRS}
  )

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(DXFLib_PROCESS_INCLUDES DXFLib_INCLUDE_DIR)
set(DXFLib_PROCESS_LIBS DXFLib_LIBRARY)
libfind_process(DXFLib)

PRINT_LIBRARY_STATUS(DXFLib
  DETAILS "[${DXFLib_LIBRARIES}][${DXFLib_INCLUDE_DIRS}]"
  )

if(DXFLib_FOUND)
  SET(DXFLib_DEFINITIONS "-D_IC_BUILDER_DXFLib_")
ENDIF()
