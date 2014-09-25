// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of FZIs ic_workspace.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2009-12-14
 *
 */
//----------------------------------------------------------------------
#include <boost/test/unit_test.hpp>
#include <icl_core_thread/Sem.h>

using icl_core::TimeSpan;
using icl_core::TimeStamp;
using icl_core::thread::Semaphore;

BOOST_AUTO_TEST_SUITE(ts_Semaphore)

BOOST_AUTO_TEST_CASE(SemaphorePostWait)
{
  Semaphore sem(0);
  sem.post();
  BOOST_CHECK(sem.wait());
}

BOOST_AUTO_TEST_CASE(SemaphoreTryWait)
{
  Semaphore sem(0);
  BOOST_CHECK(!sem.tryWait());
  sem.post();
  BOOST_CHECK(sem.tryWait());
}

BOOST_AUTO_TEST_CASE(SemaphoreWaitAbsoluteTimeout)
{
  Semaphore sem(0);
  BOOST_CHECK(!sem.wait(TimeStamp::now() + TimeSpan(1, 0)));
  sem.post();
  BOOST_CHECK(sem.wait(TimeStamp::now() + TimeSpan(1, 0)));
}

BOOST_AUTO_TEST_CASE(SemaphoreWaitRelativeTimeout)
{
  Semaphore sem(0);
  BOOST_CHECK(!sem.wait(TimeSpan(1, 0)));
  sem.post();
  BOOST_CHECK(sem.wait(TimeSpan(1, 0)));
}

BOOST_AUTO_TEST_SUITE_END()
