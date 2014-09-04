# - Try to find Newmat
# Once done, this will define
#
# Newmat_FOUND - system has Newmat
# Newmat_INCLUDE_DIRS - the Newmat include directories
# Newmat_LIBRARIES - link these to use Newmat

if (Newmat_INCLUDE_DIRS AND Newmat_LIBRARIES)
# in cache already
set(Newmat_FOUND TRUE)
else (Newmat_INCLUDE_DIRS AND Newmat_LIBRARIES)
include(LibFindMacros)

# Dependencies
# NOTE: Newmat has no explicit dependencies.
# libfind_package(Newmat Dependencies)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Newmat_PKGCONF newmat)

# Include dir
find_path(Newmat_INCLUDE_DIR
NAMES newmat/newmat.h
PATHS
${Newmat_PKGCONF_INCLUDE_DIRS}
/usr/include
/usr/local/include
)

# Finally the library itself
find_library(Newmat_LIBRARY
NAMES newmat
PATHS ${Newmat_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(Newmat_PROCESS_INCLUDES Newmat_INCLUDE_DIR)
set(Newmat_PROCESS_LIBS Newmat_LIBRARY)
libfind_process(Newmat)

if (Newmat_INCLUDE_DIRS AND Newmat_LIBRARIES)
set(Newmat_FOUND TRUE)
endif (Newmat_INCLUDE_DIRS AND Newmat_LIBRARIES)
endif (Newmat_INCLUDE_DIRS AND Newmat_LIBRARIES)