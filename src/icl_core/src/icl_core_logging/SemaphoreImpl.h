// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2009-01-20
 *
 * \brief   Contains icl_core::logging::SemaphoreImpl
 *
 * \b icl_core::logging::SemaphoreImpl
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_LOGGING_SEMAPHORE_IMPL_H_INCLUDED
#define ICL_CORE_LOGGING_SEMAPHORE_IMPL_H_INCLUDED

#include <icl_core/Noncopyable.h>

namespace icl_core {
namespace logging {

class SemaphoreImpl : protected virtual icl_core::Noncopyable
{
public:
  virtual ~SemaphoreImpl() {}
  virtual void post() = 0;
  virtual bool wait() = 0;
};

}
}

#endif
