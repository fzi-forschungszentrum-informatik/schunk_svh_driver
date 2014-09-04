# - Try to find SQLite3
# Once done this will define
#
#  SQLite3_FOUND - system has SQLite3
#  SQLite3_INCLUDE_DIR - the SQLite3 include directory
#  SQLite3_LIBRARIES - Link these to use SQLite3


IF( SQLite3_FOUND )
  # in cache already
  SET( SQLite3_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(SQLite_PKGCONF sqlite3)

# Include dir
find_path(SQLite3_INCLUDE_DIR
  NAMES sqlite3.h
  PATHS ${SQLite3_PKGCONF_INCLUDE_DIRS} "/opt/local/include"
)

# Finally the library itself
find_library(SQLite3_LIBRARY
  NAMES sqlite3
  PATHS ${SQLite3_PKGCONF_LIBRARY_DIRS} "/opt/local/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SQLite3_PROCESS_INCLUDES SQLite3_INCLUDE_DIR)
set(SQLite3_PROCESS_LIBS SQLite3_LIBRARY)
libfind_process(SQLite3)

PRINT_LIBRARY_STATUS(SQLite3
  DETAILS "[${SQLite3_LIBRARIES}][${SQLite3_INCLUDE_DIRS}]"
)
