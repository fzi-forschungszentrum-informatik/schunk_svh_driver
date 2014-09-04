 # - Try to find the Marble Library
# Once done this will define
#
#  Marble_FOUND - system has Marble
#  Marble_INCLUDE_DIR - the Marble include directory
#  Marble_LIBRARIES
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

IF( Marble_FOUND )
   # in cache already
   SET( Marble_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(Marble_PKGCONF marble)

# Include dir
find_path(Marble_INCLUDE_DIR
  NAMES marble/MarbleMap.h
  PATHS ${Marble_PKGCONF_INCLUDE_DIRS} "/usr/include"
)

# Finally the library itself
find_library(Marble_LIBRARY
  NAMES marblewidget
  PATHS ${Marble_PKGCONF_LIBRARY_DIRS} "/usr/lib"
)

set(Marble_PROCESS_INCLUDES Marble_INCLUDE_DIR)
set(Marble_PROCESS_LIBS Marble_LIBRARY)
libfind_process(Marble)

PRINT_LIBRARY_STATUS(Marble
  DETAILS "[${Marble_LIBRARIES}][${Marble_INCLUDE_DIRS}]"
)
