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

#include <driver_svh/SVHController.h>
#include <driver_svh/SVHSerialPacket.h>

using namespace driver_svh;

// testing serial interface of svh driver
int main(int argc, const char* argv[])
{
  icl_core::logging::initialize();

  std::string serial_device_name = "/dev/ttyUSB1";

  SVHController controller;
  controller.connect(serial_device_name);

  // initilize default position settings
  std::vector<SVHPositionSettings> default_position_settings(eSVH_DIMENSION);
  SVHPositionSettings pos_set_thumb(-1.0e6f, 1.0e6f,  3.4e3f, 1.00f, 1e-3f, -500.0f, 500.0f, 0.5f, 0.05f, 0.0f);
  SVHPositionSettings pos_set_finger(-1.0e6f, 1.0e6f,  8.5e3f, 1.00f, 1e-3f, -500.0f, 500.0f, 0.5f, 0.05f, 0.0f);
  SVHPositionSettings pos_set_spread(-1.0e6f, 1.0e6f, 17.0e3f, 1.00f, 1e-3f, -500.0f, 500.0f, 0.5f, 0.05f, 0.0f);

  default_position_settings[0] = pos_set_thumb;   // thumb flexion
  default_position_settings[1] = pos_set_thumb;   // thumb opposition
  default_position_settings[2] = pos_set_finger;  // index finger distal joint
  default_position_settings[3] = pos_set_finger;  // index finger proximal joint
  default_position_settings[4] = pos_set_finger;  // middle finger distal joint
  default_position_settings[5] = pos_set_finger;  // middle finger proximal joint
  default_position_settings[6] = pos_set_finger;  // ring finger
  default_position_settings[7] = pos_set_finger;  // pinky
  default_position_settings[8] = pos_set_spread;  // finger spread

  // initilize default current settings
  std::vector<SVHCurrentSettings> default_current_settings(eSVH_DIMENSION);
  SVHCurrentSettings cur_set_thumb(-191.0f, 191.0f, 0.405f, 4e-6f, -300.0f, 300.0f, 0.850f, 85.0f, -254.0f, 254.0f);
  SVHCurrentSettings cur_set_distal_joint(-176.0f, 176.0f, 0.405f, 4e-6f, -300.0f, 300.0f, 0.850f, 85.0f, -254.0f, 254.0f);
  SVHCurrentSettings cur_set_proximal_joint(-167.0f, 167.0f, 0.405f, 4e-6f, -300.0f, 300.0f, 0.850f, 85.0f, -254.0f, 254.0f);

  default_current_settings[0] = cur_set_thumb;          // thumb flexion
  default_current_settings[1] = cur_set_thumb;          // thumb opposition
  default_current_settings[2] = cur_set_distal_joint;   // index finger distal joint
  default_current_settings[3] = cur_set_proximal_joint; // index finger proximal joint
  default_current_settings[4] = cur_set_distal_joint;   // middle finger distal joint
  default_current_settings[5] = cur_set_proximal_joint; // middle finger proximal joint
  default_current_settings[6] = cur_set_distal_joint;   // ring finger
  default_current_settings[7] = cur_set_distal_joint;   // pinky
  default_current_settings[8] = cur_set_proximal_joint; // finger spread

  // request firmware info
  std::cout << "sending request firmware info" << std::endl;
  controller.requestFirmwareInfo();

  // initialize all channels
  for (size_t i = 0; i < eSVH_DIMENSION; ++i)
  {
    SVHChannel channel = SVHChannel(i);

    // request controller feedback
    std::cout << "sending request controller feedback" << std::endl;
    controller.requestControllerFeedback(channel);

    // set position settings
    std::cout << "set position settings" << std::endl;
    controller.setPositionSettings(channel, default_position_settings[channel]);
    //std::cout << "Position settings of channel " << channel << " " << default_position_settings[channel] << std::endl;

    // set current settings
    std::cout << "set current settings" << std::endl;
    controller.setCurrentSettings(channel, default_current_settings[channel]);
    //std::cout << "Current settings of channel " << channel << " " << default_current_settings[channel] << std::endl;

  }

  icl_core::os::sleep(20);

  controller.disconnect();

}
