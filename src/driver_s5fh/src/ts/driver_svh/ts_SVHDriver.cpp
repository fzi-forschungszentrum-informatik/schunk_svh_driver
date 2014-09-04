// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
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
  SVHPositionSettings test_pos_settings_in =  {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1};
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
  SVHCurrentSettings test_cur_settings_in =  {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1};
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
  SVHPositionSettings test_pos_settings_in =  {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1};
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

// This should not be an auto Testcase but rather one to execute voluntarily -> read about how to do that
//BOOST_AUTO_TEST_CASE(InitialConnectionPackets)
//{
//  std::cout << "Printing out raw Data that should be sent for a first transmission of everything:...." << std::endl;

//  // Initialize logging
//  icl_core::logging::initialize();

//  std::cout << "Deactivating all the channels..................." << std::endl;

//  // First packet -> Disable all the channels
//  SVHSerialPacket serial_packet(0,SVH_SET_CONTROLLER_STATE);
//  SVHControllerState controller_state;
//  SVHSerialInterface serial_interface(NULL);
//  SVHFingerManager finger_manager;
//  ArrayBuilder ab(40);

//  // load default position settings before the fingers are resetted
//  std::vector<SVHPositionSettings> default_position_settings
//      = finger_manager.getPositionSettingsDefaultResetParameters();

//  // load default current settings
//  std::vector<SVHCurrentSettings> default_current_settings
//      = finger_manager.getCurrentSettingsDefaultParameters();


//  controller_state.pwm_fault = 0x001F;
//  controller_state.pwm_otw   = 0x001F;

//  // default initialization to zero -> controllers are deactivated
//  ab << controller_state;
//  serial_packet.data = ab.array;
//  serial_interface.printPacketOnConsole(serial_packet);

//  // initialize all channels
//  for (size_t i = 0; i < eSVH_DIMENSION; ++i)
//  {
//    std::cout << "Requesting feedback for channel : "<< i << " ........." << std::endl;
//    // request controller feedback
//    SVHSerialPacket serial_packet1(40,SVH_GET_CONTROL_FEEDBACK|static_cast<uint8_t>(static_cast<SVHCHANNEL>(i) << 4));
//    serial_interface.printPacketOnConsole(serial_packet1);


//    std::cout << "Transmitting position settings for channel : "<< i << " ........." << std::endl;

//    // set position settings
//    SVHSerialPacket serial_packet2(0,SVH_SET_POSITION_SETTINGS|static_cast<uint8_t>(static_cast<SVHCHANNEL>(i) << 4));
//    ArrayBuilder ab2;
//    ab2 << default_position_settings[i];
//    serial_packet2.data = ab2.array;
//    serial_interface.printPacketOnConsole(serial_packet2);

//    std::cout << "Transmitting current settings for channel : "<< i << " ........." << std::endl;

//    // set current settings
//    SVHSerialPacket serial_packet3(0,SVH_SET_CURRENT_SETTINGS|static_cast<uint8_t>(static_cast<SVHCHANNEL>(i) << 4));
//    ArrayBuilder ab3;
//    ab3 << default_current_settings[i];
//    serial_packet3.data = ab3.array;
//    serial_interface.printPacketOnConsole(serial_packet3);
//  }


//  std::cout << "Done" << std::endl;


//}


// Firmware is not working yet
//BOOST_AUTO_TEST_CASE(FirmWareOutput)
//{

//  SVHFirmwareInfo firmware_info;
//  firmware_info.version_major = 23;
//  firmware_info.version_minor =42;
//  firmware_info.svh = "svh";
//  firmware_info.text = "FZI HAND CONTROL V1.1";

//  payload.reset(0);
//  payload << firmware_info;

//  std::cout << "Payload of FW:" << payload << std::endl;

//  SVHFirmwareInfo firmware_info2;
//  payload >> firmware_info2;

//  std::cout << "FW1:" << firmware_info << std::endl;
//  std::cout << "FW2:" << firmware_info2 << std::endl;
//  BOOST_CHECK_EQUAL(firmware_info,firmware_info2);
//}



BOOST_AUTO_TEST_SUITE_END()
