// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
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
