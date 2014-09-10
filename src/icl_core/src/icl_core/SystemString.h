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
 * \author  Jan Oberländer <oberlaen@fzi.de>
 * \date    2012-06-16
 *
 * \brief   Defines the SYSTEM_STRING macro.
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_SYSTEM_STRING_H_INCLUDED
#define ICL_CORE_SYSTEM_STRING_H_INCLUDED

#include <boost/preprocessor/stringize.hpp>

#define _SYSTEM_STRING_ BOOST_PP_STRINGIZE(_SYSTEM_IDENTIFIER_)

#endif
