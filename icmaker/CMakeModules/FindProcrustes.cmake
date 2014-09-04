# - Try to find Procrustes
# Once done this will define
#  Procrustes_FOUND - System has Procrustes
#  Procrustes_INCLUDE_DIRS - The Procrustes include directories

IF( Procrustes_FOUND )
  SET( Procrustes_FIND_QUIETLY TRUE )
ENDIF( Procrustes_FOUND )

INCLUDE(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Procrustes_PKGCONF procrustes)

# Include dir
find_path(Procrustes_INCLUDE_DIR
  NAMES Procrustes/qfMahalanobis.hpp
  PATHS ${Procrustes_PKGCONF_INCLUDE_DIRS} "/usr/include"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Procrustes_PROCESS_INCLUDES Procrustes_INCLUDE_DIR)
libfind_process(Procrustes)
