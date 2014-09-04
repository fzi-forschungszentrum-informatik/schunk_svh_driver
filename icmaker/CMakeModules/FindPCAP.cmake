# - Try to find PCAP
# Once done, this will define
#
#  PCAP_FOUND - system has PCAP
#  PCAP_INCLUDE_DIRS - the PCAP include directories
#  PCAP_LIBRARIES - link these to use PCAP

IF( PCAP_FOUND )
   # in cache already
   SET( PCAP_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(PCAP_PKGCONF pcap)

# Include dir
find_path(PCAP_INCLUDE_DIR
  NAMES pcap/pcap.h
  PATHS ${PCAP_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(PCAP_LIBRARY
  NAMES pcap
  PATHS ${PCAP_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(PCAP_PROCESS_INCLUDES PCAP_INCLUDE_DIR)
set(PCAP_PROCESS_LIBS PCAP_LIBRARY)
libfind_process(PCAP)

PRINT_LIBRARY_STATUS(PCAP
  DETAILS "[${PCAP_LIBRARIES}][${PCAP_INCLUDE_DIRS}]"
)

if(PCAP_FOUND)
  SET( PCAP_DEFINITIONS "-D_IC_BUILDER_PCAP_")
ENDIF()
