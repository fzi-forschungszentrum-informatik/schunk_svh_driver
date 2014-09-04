// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2012-02-15
 *
 */
//----------------------------------------------------------------------
#include "icl_core_dispatch/CallbackOperation.h"

namespace icl_core {
namespace dispatch {

CallbackOperation::CallbackOperation(boost::function<void ()> const & callback)
  : m_callback(callback)
{
}

CallbackOperation::~CallbackOperation()
{
}

void CallbackOperation::execute()
{
  m_callback();
}

}}
