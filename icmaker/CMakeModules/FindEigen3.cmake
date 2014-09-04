# - Try to find Eigen3
# Once done, this will define
#
#  Eigen3_FOUND - system has Eigen3
#  Eigen3_INCLUDE_DIRS - the Eigen3 include directories
#  Eigen3_LIBRARIES - link these to use Eigen3
# --

IF( Eigen3_FOUND )
   # in cache already
   SET( Eigen3_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Eigen3_PKGCONF eigen3)

# Include dir
find_path(Eigen3_INCLUDE_DIR
  NAMES Eigen/Core
  PATHS ${Eigen3_PKGCONF_INCLUDE_DIRS} "/usr/include/eigen3" "${CMAKE_INSTALL_PREFIX}/include/eigen3"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Eigen3_PROCESS_INCLUDES Eigen3_INCLUDE_DIR)
libfind_process(Eigen3)

IF(Eigen3_INCLUDE_DIR)
  SET(Eigen3_DEFINITIONS -D_IC_BUILDER_EIGEN_)
ENDIF()

PRINT_LIBRARY_STATUS(Eigen3
  DETAILS "[${Eigen3_INCLUDE_DIRS}]"
)

