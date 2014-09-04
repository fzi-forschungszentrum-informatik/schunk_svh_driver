# - Try to find CarMaker
# Once done this will define
#  CarMakerRealTime_FOUND - System has CarMaker
#  CarMakerRealTime_INCLUDE_DIRS - The CarMaker include directories
#  CarMakerRealTime_LIBRARIES - The libraries needed to use CarMaker
#  CarMakerRealTime_BIN_DIR - The bin dir of carmaker (used for pre-build steps)

IF( CarMakerRealTime_FOUND )
  SET( CarMakerRealTime_FIND_QUIETLY TRUE )
ENDIF( CarMakerRealTime_FOUND )

INCLUDE(LibFindMacros)
INCLUDE(PrintLibraryStatus)

FIND_PACKAGE( LibUSB REQUIRED )

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(CarMakerRealTime_PKGCONF carmakerrealtime)

# Include dir
find_path(CarMakerRealTime_INCLUDE_DIR
  NAMES CarMaker.h
  PATHS ${CarMakerRealTime_PKGCONF_INCLUDE_DIRS} "/opt/ipg/hil/linux-xeno/include"
)

find_path(CarMakerRealTime_BIN_DIR
  NAMES CreateCarMakerAppInfo
  PATHS ${CarMakerRealTime_PKGCONF_INCLUDE_DIRS} "/opt/ipg/hil/linux-xeno/bin"
)

# Libraries
find_library(CarMakerRealTime_CM_LIBRARY
  NAMES carmaker
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_C_LIBRARY
  NAMES car
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_D_LIBRARY
  NAMES ipgdriver
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_R_LIBRARY
  NAMES ipgroad
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_USB_LIBRARY
  NAMES usb
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_NATIVE_LIBRARY
  NAMES native
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_RTDM_LIBRARY
  NAMES rtdm
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_SENSO_LIBRARY
  NAMES SensoDrive
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_XENOMAI
  NAMES xenomai
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)
find_library(CarMakerRealTime_TIRE
  NAMES tametire
  PATHS ${CarMakerRealTime_PKGCONF_LIBRARY_DIRS} "/opt/ipg/hil/linux-xeno/lib"
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
# NOTE: The CM does not link against CarMakerRealTime_T_LIBRARY

set(CarMakerRealTime_PROCESS_INCLUDES CarMakerRealTime_INCLUDE_DIR )
set(CarMakerRealTime_PROCESS_LIBS 
        CarMakerRealTime_C_LIBRARY 
        CarMakerRealTime_CM_LIBRARY 
        CarMakerRealTime_D_LIBRARY 
        CarMakerRealTime_R_LIBRARY 
        CarMakerRealTime_TIRE
        CarMakerRealTime_USB_LIBRARY 
        CarMakerRealTime_NATIVE_LIBRARY 
        CarMakerRealTime_XENOMAI
        CarMakerRealTime_RTDM_LIBRARY
        CarMakerRealTime_SENSO_LIBRARY

)
libfind_process(CarMakerRealTime)

IF( DEFINED PRINT_LIBRARY_STATUS )
  PRINT_LIBRARY_STATUS(CarMakerRealTime
    DETAILS "[${CarMakerRealTime_LIBRARIES}][${CarMakerRealTime_INCLUDE_DIRS}]"
  )
ENDIF (DEFINED PRINT_LIBRARY_STATUS )
