# - Try to find MesaSR
# Once done, this will define
#
#  MesaSR_FOUND - system has MesaSR
#  MesaSR_INCLUDE_DIR - the MesaSR include directories
#  MesaSR_LIBRARY - link these to use MesaSR

include(PrintLibraryStatus)
include(LibFindMacros)

if ( MesaSR_FOUND )
   # in cache already
   SET( MesaSR_FIND_QUIETLY TRUE )
endif ()

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(MesaSR_PKGCONF libMesaSR)

# Include dir
find_path(MesaSR_INCLUDE_DIR
  NAMES libMesaSR.h
  PATHS ${MesaSR_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(MesaSR_LIBRARY
  NAMES mesasr
  PATHS ${MesaSR_PKGCONF_LIBRARY_DIRS}
)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(MesaSR_PROCESS_INCLUDES MesaSR_INCLUDE_DIR)
set(MesaSR_PROCESS_LIBS MesaSR_LIBRARY)
libfind_process(MesaSR)

PRINT_LIBRARY_STATUS(MesaSR
  DETAILS "[${MesaSR_LIBRARIES}][${MesaSR_INCLUDE_DIRS}]"
)

if(MesaSR_FOUND)
  set(MesaSR_DEFINITIONS "-D_IC_BUILDER_SWISSRANGER_")
endif()
