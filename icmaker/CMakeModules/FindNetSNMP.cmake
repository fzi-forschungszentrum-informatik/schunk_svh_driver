# - Try to find NetSNMP
# Once done, this will define
#
#  NetSNMP_FOUND - system has NetSNMP
#  NetSNMP_INCLUDE_DIR - the NetSNMP include directories
#  NetSNMP_LIBRARY - link these to use NetSNMP

if ( NetSNMP_FOUND )
   # in cache already
   SET( NetSNMP_FIND_QUIETLY TRUE )
endif ()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
# libfind_pkg_check_modules(NetSNMP_PKGCONF netsnmp)

# Include dir
find_path(NetSNMP_INCLUDE_DIR
  NAMES net-snmp/net-snmp-config.h
  PATHS ${NetSNMP_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(NetSNMP_LIBRARY
  NAMES netsnmp
  PATHS ${NetSNMP_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(NetSNMP_PROCESS_INCLUDES NetSNMP_INCLUDE_DIR)
set(NetSNMP_PROCESS_LIBS NetSNMP_LIBRARY)
libfind_process(NetSNMP)

PRINT_LIBRARY_STATUS(NetSNMP
  DETAILS "[${NetSNMP_LIBRARIES}][${NetSNMP_INCLUDE_DIRS}]"
)
