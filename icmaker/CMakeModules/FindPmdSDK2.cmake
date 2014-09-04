# - Try to find PmdSDK2
# Once done, this will define
#
#  PmdSDK2_FOUND - system has PmdSDK2
#  PmdSDK2_INCLUDE_DIR - the PmdSDK2 include directories
#  PmdSDK2_LIBRARY - link these to use PmdSDK2

include(PrintLibraryStatus)
include(LibFindMacros)

if ( PmdSDK2_FOUND )
   # in cache already
   SET( PmdSDK2_FIND_QUIETLY TRUE )
endif ()

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(PmdSDK2_PKGCONF libPmdSDK2)

# Include dir
find_path(PmdSDK2_INCLUDE_DIR
  NAMES pmdsdk2.h
  PATHS ${PmdSDK2_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(PmdSDK2_LIBRARY
  NAMES pmdaccess2
  PATHS ${PmdSDK2_PKGCONF_LIBRARY_DIRS}
)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(PmdSDK2_PROCESS_INCLUDES PmdSDK2_INCLUDE_DIR)
set(PmdSDK2_PROCESS_LIBS PmdSDK2_LIBRARY)
libfind_process(PmdSDK2)

PRINT_LIBRARY_STATUS(PmdSDK2
  DETAILS "[${PmdSDK2_LIBRARIES}][${PmdSDK2_INCLUDE_DIRS}]"
)

if(PmdSDK2_FOUND)
  set(PmdSDK2_DEFINITIONS "-D_IC_BUILDER_PMDSDK2_")
endif()
