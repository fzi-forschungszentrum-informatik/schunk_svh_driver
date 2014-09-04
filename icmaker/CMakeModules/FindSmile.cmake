# - Try to find Smile
# Once done, this will define
#
#  Smile_FOUND - system has Smile
#  Smile_INCLUDE_DIR - the Smile include directories
#  Smile_LIBRARY - link these to use Smile

include(PrintLibraryStatus)
include(LibFindMacros)

#if ( Smile_FOUND )
#   # in cache already
#   SET( Smile_FIND_QUIETLY TRUE )
#endif ()

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Smile_PKGCONF libsmile)

IF (NOT Smile_FOUND)
  SET (Smile_PKGCONF_INCLUDE_DIRS "/opt/tools/smile")
  SET (Smile_PKGCONF_LIBRARY_DIRS "/opt/tools/smile")
ENDIF ()


# Include dir
find_path(Smile_INCLUDE_DIR
  NAMES smile.h
  PATHS ${Smile_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(Smile_LIBRARY
  NAMES smile
  PATHS ${Smile_PKGCONF_LIBRARY_DIRS}
)


# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Smile_PROCESS_INCLUDES Smile_INCLUDE_DIR)
set(Smile_PROCESS_LIBS Smile_LIBRARY)
libfind_process(Smile)

PRINT_LIBRARY_STATUS(Smile
  DETAILS "[${Smile_LIBRARIES}][${Smile_INCLUDE_DIRS}]"
)

if(Smile_FOUND)
  set(Smile_DEFINITIONS "-D_IC_BUILDER_SMILE_")
endif()
