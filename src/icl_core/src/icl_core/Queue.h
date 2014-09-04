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
#ifndef ICL_CORE_QUEUE_H_INCLUDED
#define ICL_CORE_QUEUE_H_INCLUDED

#include <queue>

namespace icl_core {

// \todo Create a wrapper class (and/or additional RT-safe implementations).
template <typename T>
class Queue : public std::queue<T>
{
public:
  Queue() : std::queue<T>() { }
  Queue(const Queue& c) : std::queue<T>(c) { }
  Queue(const std::queue<T>& c) : std::queue<T>(c) { }
};

}

#endif
