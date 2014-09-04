// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jan Oberländer <oberlaen@fzi.de>
 * \date    2009-06-16
 *
 * \brief   Contains a basic threading model for icl_core::Singleton.
 *
 * \b icl_core::STMSingleThreaded is the simple, single-threaded
 * threading model for icl_core::Singleton.  It performs no locking.
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_SINGLETON_THREADING_MODELS_H_INCLUDED
#define ICL_CORE_SINGLETON_THREADING_MODELS_H_INCLUDED

namespace icl_core {

//! Dummy threading model for single-threaded environments.
template
<class T>
class STMSingleThreaded
{
public:
  //! Memory barrier for synchronization.
  static inline void memoryBarrier() { }

  //! A dummy type for RAII locking.
  typedef int Guard;

  //! A dummy type for locking.
  typedef int Lock;
};

}

#endif
