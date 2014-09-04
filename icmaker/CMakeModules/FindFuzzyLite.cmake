# - Try to find FuzzyLite
# Once done, this will define
#
#  FuzzyLite_FOUND - system has FuzzyLite
#  FuzzyLite_INCLUDE_DIRS - the FuzzyLite include directories
#  FuzzyLite_LIBRARIES - link these to use FuzzyLite

IF( FuzzyLite_FOUND )
   # in cache already
   SET( FuzzyLite_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(FuzzyLite_PKGCONF libgps)

# Include dir
find_path(FuzzyLite_INCLUDE_DIR
  NAMES fuzzylite/FuzzyLite.h
  PATHS ${FuzzyLite_PKGCONF_INCLUDE_DIRS} "/opt/local/include" "/opt/tools/fuzzylite-i386/include"
)

# Finally the library itself
find_library(FuzzyLite_LIBRARY
  NAMES fuzzylite
  PATHS ${FuzzyLite_PKGCONF_LIBRARY_DIRS} "/opt/local/lib" "/opt/tools/fuzzylite-i386/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(FuzzyLite_PROCESS_INCLUDES FuzzyLite_INCLUDE_DIR)
set(FuzzyLite_PROCESS_LIBS FuzzyLite_LIBRARY)
libfind_process(FuzzyLite)

PRINT_LIBRARY_STATUS(FuzzyLite
  DETAILS "[${FuzzyLite_LIBRARIES}][${FuzzyLite_INCLUDE_DIRS}]"
)

