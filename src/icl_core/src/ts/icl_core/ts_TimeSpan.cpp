// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2009-12-14
 *
 */
//----------------------------------------------------------------------
#include <icl_core/TimeSpan.h>

#include <boost/test/unit_test.hpp>

using icl_core::TimeSpan;

BOOST_AUTO_TEST_SUITE(ts_TimeSpan)

BOOST_AUTO_TEST_CASE(ConstructFromValue)
{
  TimeSpan time_span(100, 55);
  BOOST_CHECK_EQUAL(time_span.tsSec(), 100);
  BOOST_CHECK_EQUAL(time_span.tsNSec(), 55);
}

BOOST_AUTO_TEST_CASE(ConstructFromTimespec)
{
  struct timespec time_spec;
  time_spec.tv_sec = 100;
  time_spec.tv_nsec = 55;
  TimeSpan time_span(time_spec);
  BOOST_CHECK_EQUAL(time_span.tsSec(), 100);
  BOOST_CHECK_EQUAL(time_span.tsNSec(), 55);
}

BOOST_AUTO_TEST_SUITE_END()
