# - Try to find SSL
# Once done, this will define
#
#  SSL_FOUND - system has SSL
#  SSL_INCLUDE_DIRS - the SSL include directories
#  SSL_LIBRARIES - link these to use SSL

IF(SSL_FOUND)
   # in cache already
   SET( SSL_FIND_QUIETLY TRUE )
ENDIF()

include(PrintLibraryStatus)
include(LibFindMacros)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(SSL_PKGCONF ssl)

# Include dir
find_path(SSL_INCLUDE_DIR
  NAMES openssl/md5.h
  PATHS ${SSL_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(SSL_LIBRARY
  NAMES ssl
  PATHS ${SSL_PKGCONF_LIBRARY_DIRS}
)

# Finally the library itself
find_library(Crypto_LIBRARY
  NAMES crypto
  PATHS ${SSL_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(SSL_PROCESS_INCLUDES SSL_INCLUDE_DIR)
set(SSL_PROCESS_LIBS SSL_LIBRARY Crypto_LIBRARY)
libfind_process(SSL)

PRINT_LIBRARY_STATUS(SSL
  DETAILS "[${SSL_LIBRARIES}][${SSL_INCLUDE_DIRS}]"
)
