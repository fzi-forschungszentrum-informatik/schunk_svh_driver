# - Try to find LibSVM
# Once done, this will define
#
#  SVM_FOUND - system has LibSVM
#  SVM_INCLUDE_DIR - the LibSVM include directories
#  SVM_LIBRARY - link these to use LibSVM

if ( SVM_FOUND )
   # in cache already
   SET( SVM_FIND_QUIETLY TRUE )
endif ()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(SVM_PKGCONF svm)

# Include dir
find_path(SVM_INCLUDE_DIR
  NAMES svm.h
  PATHS ${SVM_PKGCONF_INCLUDE_DIRS} "/usr/include/libsvm/"
)

# Finally the library itself
find_library(SVM_LIBRARY
  NAMES svm
  PATHS ${SVM_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SVM_PROCESS_INCLUDES SVM_INCLUDE_DIR)
set(SVM_PROCESS_LIBS SVM_LIBRARY)
libfind_process(SVM)

PRINT_LIBRARY_STATUS(SVM
  DETAILS "[${SVM_LIBRARIES}][${SVM_INCLUDE_DIRS}]"
)
