# - Try to find Eigen2
# Once done, this will define
#
#  Eigen2_FOUND - system has Eigen2
#  Eigen2_INCLUDE_DIRS - the Eigen2 include directories
#  Eigen2_LIBRARIES - link these to use Eigen2
# --

IF( Eigen2_FOUND )
   # in cache already
   SET( Eigen2_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Eigen2_PKGCONF eigen2)

# Include dir
find_path(Eigen2_INCLUDE_DIR
  NAMES Eigen/Core
  PATHS ${Eigen2_PKGCONF_INCLUDE_DIRS} "/usr/include/eigen2" "${CMAKE_INSTALL_PREFIX}/include/eigen2"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Eigen2_PROCESS_INCLUDES Eigen2_INCLUDE_DIR)
libfind_process(Eigen2)

IF(Eigen2_INCLUDE_DIR)
  SET(Eigen2_DEFINITIONS -D_IC_BUILDER_EIGEN_)
ENDIF()

PRINT_LIBRARY_STATUS(Eigen2
  DETAILS "[${Eigen2_INCLUDE_DIRS}]"
)
