// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2008-04-16
 *
 * \brief   Contains icl_core::tQueue
 *
 * \b icl_core::tQueue
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_T_QUEUE_H_INCLUDED
#define ICL_CORE_T_QUEUE_H_INCLUDED

#include <queue>

#include "icl_core/Deprecate.h"

namespace icl_core {

// \todo Create a wrapper class (and/or additional RT-safe implementations).
template <typename T>
class ICL_CORE_VC_DEPRECATE tQueue : public std::queue<T>
{
} ICL_CORE_GCC_DEPRECATE;

}

#endif
