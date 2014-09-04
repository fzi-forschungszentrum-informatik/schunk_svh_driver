# Example CMakeLists.txt which builds a library.
ICMAKER_SET("icl_core_sourcesink_gui")

# Add your cpp files here.
ICMAKER_ADD_SOURCES(
  TimeControlWidget.cpp
  )

# Add all .h and .hpp files here.
ICMAKER_ADD_HEADERS(
  ImportExport.h
  TimeControlWidget.h
  )

ICMAKER_LOCAL_CPPDEFINES(-DICL_CORE_SOURCESINK_GUI_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_ICL_CORE_SOURCESINK_GUI_)
ICMAKER_INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_BUILD_DIR}
)


ICMAKER_DEPENDENCIES(EXPORT
  icl_core_sourcesink
  QT_QTCORE
  QT_QTGUI
)

ICMAKER_ADD_QT_UI_FILES(TimeControl.ui)

IF (QT4_FOUND)
  QT4_AUTOMOC(TimeControlWidget.cpp)
ENDIF()


IF("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows")
  MESSAGE(WARNING "Not building icl_core_sourcesink_gui on Windows")
ENDIF()

IF(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  ICMAKER_BUILD_LIBRARY()
  ICMAKER_INSTALL_HEADERS(${icmaker_target})
ENDIF()

