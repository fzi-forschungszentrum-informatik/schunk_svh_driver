// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2009-02-10
 *
 * \brief   Contains icl_core::Noncopyable
 *
 * \b icl_core::Noncopyable
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_NONCOPYABLE_H_INCLUDED
#define ICL_CORE_NONCOPYABLE_H_INCLUDED

#include "icl_core/ImportExport.h"

namespace icl_core {

class ICL_CORE_IMPORT_EXPORT Noncopyable
{
protected:
  Noncopyable();
  virtual ~Noncopyable();

private:  // emphasize the following members are private
  Noncopyable(const Noncopyable&);
  const Noncopyable& operator = (const Noncopyable&);
};

}

#endif
