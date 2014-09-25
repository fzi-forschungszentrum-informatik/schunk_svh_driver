// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of FZIs ic_workspace.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
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
#include "icl_core_config/ConfigPositionalParameter.h"

namespace icl_core {
namespace config {

ConfigPositionalParameter::ConfigPositionalParameter(const String &name,
                                                     const String &config_key,
                                                     const String &help,
                                                     const bool is_optional,
                                                     const String &default_value)
  : GetoptPositionalParameter(name,
                              default_value.empty() ? help : help + "\n(defaults to " + default_value + ")",
                              is_optional),
    m_config_key(config_key),
    m_default_value(default_value)
{

}

}
}
