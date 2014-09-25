// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 SCHUNK Mobile Greifsysteme GmbH, Lauffen/Neckar Germany
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \date    2014-02-15
 *
 */
//----------------------------------------------------------------------

#include <driver_svh/SVHFingerManager.h>
#include <driver_svh/SVHSerialPacket.h>

using namespace driver_svh;

// testing serial interface of svh driver
int main(int argc, const char* argv[])
{
  icl_core::logging::initialize();

  std::string serial_device_name = "/dev/ttyUSB1";

  SVHFingerManager finger_manager;
  if (finger_manager.connect(serial_device_name))
  {
    SVHChannel test_channel = eSVH_ALL;

    std::cout << "connected" << std::endl;

    finger_manager.resetChannel(test_channel);

    icl_core::os::sleep(2);

    std::cout << "Pinky is enabled: " << finger_manager.isEnabled(test_channel) << std::endl;
    std::cout << "Pinky is homed: " << finger_manager.isHomed(test_channel) << std::endl;

    std::cout << "Enabling Pinky: " << finger_manager.enableChannel(test_channel) << std::endl;


    icl_core::os::sleep(2);

    std::cout << "Pinky is enabled: " << finger_manager.isEnabled(test_channel) << std::endl;
    std::cout << "Pinky is homed: " << finger_manager.isHomed(test_channel) << std::endl;


    icl_core::os::sleep(10);

    std::cout << "after sleep" << std::endl;

    finger_manager.disconnect();
  }

}
