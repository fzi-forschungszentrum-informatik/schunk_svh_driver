# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
ICMAKER_SET("icl_core_sourcesink" IDE_FOLDER ${ICL_CORE_IDE_FOLDER})

ICMAKER_ADD_SOURCES(
  Logger.cpp

  DataSourceSink.cpp
  DataSourceBase.cpp

  Traits.cpp
  DataHeader.cpp
  PositionResolver.cpp
  PositionResolverBinary.cpp
  Position.cpp
)

ICMAKER_ADD_HEADERS(
  ImportExport.h
  Logger.h
  Exception.h

  DataSourceSink.h
  DataSourceBase.h

  DataSink.h
  DataSink.hpp

  DataSource.h
  DataSource.hpp
  DataSourceIterator.h
  DataSourceIterator.hpp
  SeekableDataSource.h
  SeekableDataSource.hpp
  LiveDataSource.h
  LiveDataSource.hpp

  DataFilter.h
  DataFilter.hpp

  Traits.h
  DataHeader.h
  PositionResolver.h
  PositionResolverBinary.h
  PositionResolverExtra.h
  PositionResolverExtra.hpp
  Position.h
)

ICMAKER_LOCAL_CPPDEFINES(-DICL_CORE_SOURCESINK_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_ICL_CORE_SOURCESINK_)
ICMAKER_INCLUDE_DIRECTORIES(${ICL_CORE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  Boost
  Boost_DATE_TIME
  Boost_SERIALIZATION
  icl_core_logging
)

ICMAKER_BUILD_LIBRARY()
#ICMAKER_BUILD_CUSTOM("Includes" "${icl_core_sourcesink_HEADERS}")
ICMAKER_INSTALL_HEADERS("icl_core_sourcesink")
