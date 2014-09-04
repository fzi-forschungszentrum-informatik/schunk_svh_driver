// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2011-04-07
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_CORE_MAP_H_INCLUDED
#define ICL_CORE_MAP_H_INCLUDED

#include <map>

namespace icl_core
{

// \todo Create a wrapper class (and/or additional RT-safe implementations).
template <typename TKey, typename TValue>
class Map : public std::map<TKey, TValue>
{
public:
  Map() : std::map<TKey, TValue>() { }
  Map(const Map& c) : std::map<TKey, TValue>(c) { }
  Map(const std::map<TKey, TValue>& c) : std::map<TKey, TValue>(c) { }
  template <typename TInputIterator>
  Map(TInputIterator start, TInputIterator end) : std::map<TKey, TValue>(start, end) { }
};

}

#endif
