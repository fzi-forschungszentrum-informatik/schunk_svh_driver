// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2011-11-07
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_SET_H_INCLUDED
#define ICL_CORE_SET_H_INCLUDED

#include <set>

namespace icl_core {

// \todo Create a wrapper class (and/or additional RT-safe implementations).
template <typename T>
class Set : public std::set<T>
{
public:
  Set() : std::set<T>() { }
  Set(const Set& c) : std::set<T>(c) { }
  Set(const std::set<T>& c) : std::set<T>(c) { }
};

}

#endif
