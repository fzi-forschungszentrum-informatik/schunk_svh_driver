#include "Logger.h"


/** \file Logger.h
 * Contains SourceSinkLogger registration and library version information
 *
 * @author Ralf Kohlhaas <kohlhaas@fzi.de>
 * @date 2013-10-16
 */

namespace icl_core {
namespace source_sink {

std::string version()
{
  return std::string("no version");
}

std::string Version()
{
  return version();
}

REGISTER_LOG_STREAM_OPERATOR( boost::posix_time::ptime)

REGISTER_LOG_STREAM( SourceSinkLogger)

}
}
