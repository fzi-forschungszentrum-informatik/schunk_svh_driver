# - Try to find HbFoundationExtract
# Once done, this will define
#
#  HbFoundationExtract_FOUND - system has HbFoundationExtract
#  HbFoundationExtract_INCLUDE_DIRS - the HbFoundationExtract include directories
#  HbFoundationExtract_LIBRARIES - link these to use HbFoundationExtract

include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(HbFoundationExtract_PKGCONF HbFoundationExtract)

# Include dir
find_path(HbFoundationExtract_INCLUDE_DIR
  NAMES HBSTypes.hpp
  PATHS ${HbFoundationExtract_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(HbFoundationExtract_LIBRARY
  NAMES hb_foundation_extract
  PATHS ${HbFoundationExtract_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(HbFoundationExtract_PROCESS_INCLUDES HbFoundationExtract_INCLUDE_DIR)
set(HbFoundationExtract_PROCESS_LIBS HbFoundationExtract_LIBRARY)
libfind_process(HbFoundationExtract)

if(HbFoundationExtract_FOUND)

endif()
