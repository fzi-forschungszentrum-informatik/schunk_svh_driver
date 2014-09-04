// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2010-06-30
 *
 * \brief   Contains icl_core::thread::SemaphoreImplLxrt38
 *
 * \b icl_core::thread::SemaphoreImplLxrt38
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_THREAD_SEMAPHORE_IMPL_LXRT38_H_INCLUDED
#define ICL_CORE_THREAD_SEMAPHORE_IMPL_LXRT38_H_INCLUDED

#include <rtai_sem.h>

#include "icl_core/BaseTypes.h"
#include "SemaphoreImpl.h"

namespace icl_core {
namespace thread {

class SemaphoreImplLxrt38 : public SemaphoreImpl, protected virtual icl_core::Noncopyable
{
public:
  SemaphoreImplLxrt38(size_t initial_value, int type = CNT_SEM);
  virtual ~SemaphoreImplLxrt38();

  virtual void post();
  virtual bool tryWait();
  virtual bool wait();
  virtual bool wait(const icl_core::TimeSpan& timeout);
  virtual bool wait(const icl_core::TimeStamp& timeout);

private:
  SEM *m_semaphore;
};

}
}

#endif
