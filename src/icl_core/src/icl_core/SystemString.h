// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
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
