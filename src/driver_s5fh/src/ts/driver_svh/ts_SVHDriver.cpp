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
 * \author  Georg Heppner
 * \date    2014-02-03
 *
 */
//----------------------------------------------------------------------
#include <icl_comm/ByteOrderConversion.h>
#include <boost/test/unit_test.hpp>

#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHSerialPacket.h>
#include <driver_svh/SVHControllerState.h>
#include <driver_svh/SVHControlCommand.h>
#include <driver_svh/SVHControllerFeedback.h>
#include <driver_svh/SVHEncoderSettings.h>
#include <driver_svh/SVHController.h>
#include <driver_svh/SVHFingerManager.h>

using icl_comm::ArrayBuilder;
using namespace driver_svh;

BOOST_AUTO_TEST_SUITE(ts_SVHDriver)

ArrayBuilder payload(40);

BOOST_AUTO_TEST_CASE(ConvertPosSettings)
{
  std::cout << "Conversion test of Position Settings ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHPositionSettings test_pos_settings_in(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1);
  SVHPositionSettings test_pos_settings_out;

  // Conversion
  payload << test_pos_settings_in;
  payload >> test_pos_settings_out;

  BOOST_CHECK_EQUAL(test_pos_settings_in,test_pos_settings_out);

  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ConvertCurSettings)
{
  std::cout << "Conversion test of Current Settings ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHCurrentSettings test_cur_settings_in(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1);
  SVHCurrentSettings test_cur_settings_out;

  // Conversion
  payload << test_cur_settings_in;
  payload >> test_cur_settings_out;

  BOOST_CHECK_EQUAL(test_cur_settings_in,test_cur_settings_out);

  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ConvertControlCommand)
{
  std::cout << "Conversion test of Controll Command ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHControlCommand test_control_command_in(23);
  SVHControlCommand test_control_command_out(0);

  // Conversion
  payload << test_control_command_in;
  payload >> test_control_command_out;

  BOOST_CHECK_EQUAL(test_control_command_in,test_control_command_out);

  std::cout << "Done" << std::endl;
}


BOOST_AUTO_TEST_CASE(ConvertControlFeedback)
{
  std::cout << "Conversion test of ControllerFeedback ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHControllerFeedback test_controller_feedback_in(23,42);
  SVHControllerFeedback test_controller_feedback_out;

  // Conversion
  payload << test_controller_feedback_in;
  payload >> test_controller_feedback_out;

  BOOST_CHECK_EQUAL(test_controller_feedback_in,test_controller_feedback_out);

  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ConvertControllerState)
{
  std::cout << "Converstion test of ControllerState ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHControllerState test_controller_state_in(0x001F,0x001F,0x0200,0x02000,0x0001,0x0001);
  SVHControllerState test_controller_state_out;

  // Conversion
  payload << test_controller_state_in;
  payload >> test_controller_state_out;



  BOOST_CHECK_EQUAL(test_controller_state_in,test_controller_state_out);

  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ConvertEncoderSettings)
{
  std::cout << "Converstion test of EncoderSettings ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHEncoderSettings test_encoder_settings_in(23);
  SVHEncoderSettings test_encoder_settings_out;

  // Conversion
  payload << test_encoder_settings_in;
  payload >> test_encoder_settings_out;

  BOOST_CHECK_EQUAL(test_encoder_settings_in,test_encoder_settings_out);

  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ConvertControlCommandAllChannels)
{
  std::cout << "Converstion test of ControlCommandAllChannels ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHControlCommandAllChannels test_control_command_in(0,1,2,3,4,5,6,7,8);
  SVHControlCommandAllChannels test_control_command_out;

    // Conversion
  payload << test_control_command_in;
  payload >> test_control_command_out;

  BOOST_CHECK_EQUAL(test_control_command_in,test_control_command_out);

  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ConvertControllerFeedbackAllChannels)
{
  std::cout << "Converstion test of ControllerFeedbackAllChannels ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  SVHControllerFeedbackAllChannels test_controller_feedback_all_channels_in(SVHControllerFeedback(0,9),
                                                                             SVHControllerFeedback(1,10),
                                                                             SVHControllerFeedback(2,11),
                                                                             SVHControllerFeedback(3,12),
                                                                             SVHControllerFeedback(4,13),
                                                                             SVHControllerFeedback(5,14),
                                                                             SVHControllerFeedback(6,15),
                                                                             SVHControllerFeedback(7,16),
                                                                             SVHControllerFeedback(8,17));
  SVHControllerFeedbackAllChannels test_controller_feedback_all_channels_out;



  // Conversion
  payload << test_controller_feedback_all_channels_in;
  payload >> test_controller_feedback_all_channels_out;

  BOOST_CHECK_EQUAL(test_controller_feedback_all_channels_in,test_controller_feedback_all_channels_out);

  std::cout << "Done" << std::endl;
}


BOOST_AUTO_TEST_CASE(ConvertSerialPacket)
{

  std::cout << "Converstion test of Serial Packet ....";

  // Reset Array Builder
  payload.reset(40);

  // Create Structures
  ArrayBuilder packet;
  SVHSerialPacket test_serial_packet_in(40,5);
  SVHSerialPacket test_serial_packet_out(40);
  SVHPositionSettings test_pos_settings_in(0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1);
  SVHPositionSettings test_pos_settings_out;

  // Conversion
  payload << test_pos_settings_in;
  // Insertion
  test_serial_packet_in.data = payload.array;
  // Converstion (this would be done by serialinterface)
  packet << test_serial_packet_in;

 // std::cout << "Raw Serial Packet of Position settings: " << packet << std::endl;

  packet >> test_serial_packet_out;
  payload.reset(40);
  payload.appendWithoutConversion(test_serial_packet_out.data);
  payload >> test_pos_settings_out;

  BOOST_CHECK_EQUAL(test_serial_packet_in,test_serial_packet_out);
  BOOST_CHECK_EQUAL(test_pos_settings_in,test_pos_settings_out);

  //std::cout << "Original pos Settings: " << test_pos_settings_in << " Interpreted pos settings: "<< test_pos_settings_out <<std::endl;
  std::cout << "Done" << std::endl;
}

BOOST_AUTO_TEST_CASE(ControllerreceiveFeedback)
{

  std::cout << "Controller receiving feedback Packet ....";

  // Initialize logging
  icl_core::logging::initialize();

  // Reset Array Builder
  // BEWARE OF ARRAY LENGTH!
  payload.reset(64);

  // Create Structures
  SVHController controller;
  ArrayBuilder packet;
  SVHChannel channel = eSVH_INDEX_FINGER_DISTAL;
  SVHSerialPacket test_serial_packet(64,SVH_SET_CONTROL_COMMAND|static_cast<uint8_t>(channel << 4));
  SVHControllerFeedback test_controller_feedback(23,42);
  // Conversion
  payload << test_controller_feedback;


  // Insertion
  test_serial_packet.data = payload.array;

  // Emulate received packet
  controller.receivedPacketCallback(test_serial_packet,1);

  // Get packet from controller
  SVHControllerFeedback feedback_out;
  controller.getControllerFeedback(channel,feedback_out);

  BOOST_CHECK_EQUAL(test_controller_feedback,feedback_out);

  std::cout << "Done" << std::endl;
}



BOOST_AUTO_TEST_SUITE_END()
