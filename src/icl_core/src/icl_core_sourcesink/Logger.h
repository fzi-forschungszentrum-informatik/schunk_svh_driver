#ifndef ICL_CORE_SOURCE_SINK_LOGGER_H
#define ICL_CORE_SOURCE_SINK_LOGGER_H

/** \file Logger.h
 * Contains SourceSinkLogger definition
 *
 * @author Ralf Kohlhaas <kohlhaas@fzi.de>
 * @date 2013-10-16
 */

#include <icl_core_logging/Logging.h>
#include "ImportExport.h"

#include "boost/date_time/posix_time/posix_time.hpp"
#include <string>

namespace icl_core {
namespace source_sink {


ICL_CORE_SOURCESINK_IMPORT_EXPORT std::string Version();

ICL_CORE_SOURCESINK_IMPORT_EXPORT DECLARE_LOG_STREAM_OPERATOR(boost::posix_time::ptime)

DECLARE_LOG_STREAM_IMPORT_EXPORT(SourceSinkLogger, ICL_CORE_SOURCESINK_IMPORT_EXPORT)

}
}

#define SOURCE_SINK_ERROR(message) LOGGING_ERROR(icl_core::source_sink::SourceSinkLogger, message << icl_core::logging::endl);
#define SOURCE_SINK_WARN(message) LOGGING_WARNING(icl_core::source_sink::SourceSinkLogger, message << icl_core::logging::endl);
#define SOURCE_SINK_INFO(message) LOGGING_INFO(icl_core::source_sink::SourceSinkLogger, message << icl_core::logging::endl);
#define SOURCE_SINK_DEBUG(message) LOGGING_DEBUG(icl_core::source_sink::SourceSinkLogger, message << icl_core::logging::endl);
#define SOURCE_SINK_TRACE(message) LOGGING_TRACE(icl_core::source_sink::SourceSinkLogger, message << icl_core::logging::endl);

#endif // ICL_CORE_SOURCE_SINK_LOGGER_H
