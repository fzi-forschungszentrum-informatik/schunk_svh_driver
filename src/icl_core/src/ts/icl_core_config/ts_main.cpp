// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2012-01-25
 *
 */
//----------------------------------------------------------------------
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>

#include <icl_core/os_lxrt.h>
#include <icl_core_logging/Logging.h>

struct GlobalFixture
{
  GlobalFixture()
  {
    icl_core::os::lxrtStartup();
    icl_core::logging::initialize();
  }

  ~GlobalFixture()
  {
    icl_core::logging::shutdown();
    icl_core::os::lxrtShutdown();
  }
};

BOOST_GLOBAL_FIXTURE(GlobalFixture)
