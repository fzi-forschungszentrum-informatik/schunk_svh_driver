// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Florian Kuhnt <kuhnt@fzi.de>
 * \date    2014-05-07
 *
 */
//----------------------------------------------------------------------
#include "icl_core_config/GetoptPositionalParameter.h"

namespace icl_core {
namespace config {

GetoptPositionalParameter::GetoptPositionalParameter(const String &name, const String &help, const bool is_optional)
  : m_name(name),
    m_help(help),
    m_is_optional(is_optional)
{

}

}
}
