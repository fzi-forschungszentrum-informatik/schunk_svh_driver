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
 * \author  Lars Pfotzer
 * \date    2014-01-30
 * \date    2014-07-20
 *
 * This file contains the SVH controler, the heart of the driver.
 * It is responsible to manage all logical decissions regarding the hardware
 * on a low level. It knows what packet index is used for which function
 * and holds all the data objects that can be queried externally.
 * The controller should not be queried by outside calls directly as it asumes
 * the calls to be non maleformed or contain out of bounds acces as this is handled
 * by the finger manager. Also note that the calls on this level should be made
 * channel wise. The iteration of channels is done in the finger controller.
 *
 * Request and Get principle: As the communication with the hand had some issues with the bandwith
 * there are two types of function calls. The request functions tell the driver to actually request the data
 * from the hardware. The get functions just get the last received value from the controller without actually
 * querrying the hardware. This might be changed in further releases.
 */
//----------------------------------------------------------------------
#include "driver_svh/SVHController.h"

#include <driver_svh/Logging.h>
#include <icl_comm/ByteOrderConversion.h>
#include <boost/bind/bind.hpp>

using icl_comm::ArrayBuilder;

namespace driver_svh {


//! Description of the channels for enum matching:
const char * SVHController::m_channel_description[]= {
  "Thumb_Flexion",
  "Thumb_Opposition", // wrist
  "Index_Finger_Distal",
  "Index_Finger_Proximal",
  "Middle_Finger_Distal",
  "Middle_Finger_Proximal",
  "Ring_Finger",
  "Pinky",
  "Finger_Spread",
  NULL
};

SVHController::SVHController():
  m_current_settings(eSVH_DIMENSION),  // Vectors have to be filled with objects for correct deserialization
  m_position_settings(eSVH_DIMENSION),
  m_controller_feedback(eSVH_DIMENSION),
  m_serial_interface(new SVHSerialInterface(boost::bind(&SVHController::receivedPacketCallback,this,_1,_2))),
  m_enable_mask(0),
  m_received_package_count(0)
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "SVH Controller started"<< endl);
}

SVHController::~SVHController()
{
  if (m_serial_interface != NULL)
  {
    disconnect();
    delete m_serial_interface;
    m_serial_interface = NULL;
  }

  LOGGING_DEBUG_C(DriverSVH, SVHController, "SVH Controller terminated" << endl);
}

bool SVHController::connect(const std::string &dev_name)
{
  LOGGING_DEBUG_C(DriverSVH, SVHController, "Connect was called, starting the serial interface..." << endl);
  if (m_serial_interface != NULL)
  {
    LOGGING_DEBUG_C(DriverSVH, SVHController, "Connect finished succesfully" << endl);
    return m_serial_interface->connect(dev_name);
  }
  else
  {
    LOGGING_DEBUG_C(DriverSVH, SVHController, "Connect failed" << endl);
    return false;
  }
}

void SVHController::disconnect()
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Disconnect called, disabling all channels and closing interface..."<< endl);
  if (m_serial_interface != NULL && m_serial_interface->isConnected())
  {
    // Disable all channels
    disableChannel(eSVH_ALL);
    m_serial_interface->close();
  }
  LOGGING_TRACE_C(DriverSVH, SVHController, "Disconnect finished"<< endl);
}

void SVHController::setControllerTarget(const SVHChannel& channel, const int32_t& position)
{
  // No Sanity Checks for out of bounds positions at this point as the finger manager has already handled these
  if ((channel != eSVH_ALL) && (channel >=0 && channel < eSVH_DIMENSION))
  {
    // The channel is encoded in the index byte
    SVHSerialPacket serial_packet(0,SVH_SET_CONTROL_COMMAND|static_cast<uint8_t>(channel << 4));
    SVHControlCommand control_command(position);
    // Note the 40 byte ArrayBuilder initialization -> this is needed to get a zero padding in the serialpacket. Otherwise it would be shorter
    ArrayBuilder ab(40);
    ab << control_command;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);

    // Debug Disabled as it is way to noisy
    //LOGGING_TRACE_C(DriverSVH, SVHController, "Control command was given for channel: "<< channel << "Driving motor to position: "<< position << endl);
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Control command was given for unknown (or all) channel: "<< channel << "- ignoring request"<< endl);
  }
}

void SVHController::setControllerTargetAllChannels(const std::vector<int32_t> &positions)
{
  if (positions.size() >= eSVH_DIMENSION)
  {
   SVHSerialPacket serial_packet(0,SVH_SET_CONTROL_COMMAND_ALL);
   SVHControlCommandAllChannels control_command(positions);
   ArrayBuilder ab(40);
   ab << control_command;
   serial_packet.data = ab.array;
   m_serial_interface ->sendPacket(serial_packet);

   // Debug Disabled as it is way to noisy
   //LOGGING_TRACE_C(DriverSVH, SVHController, "Control command was given for all channels: Driving motors to positions: "<< positions[0] << " , " << positions[1] << " , " << positions[2] << " , " << positions[3] << " , " << positions[4] << " , " << positions[5] << " , " << positions[6] << " , " << positions[7] << " , " << positions[8] << " , " << endl);
  }
  // We could theoretically allow fewer channels but this leaves some questions. Are the given channels in right order?
  // was it realy intented to just give fewer positions? What to do witht the ones that did not get anything?
  // Just disallowing it seems to be the most understandable decission.
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Control command was given for all channels but with to few points. Expected at least "<< eSVH_DIMENSION << " values but only got " << positions.size() << "use the individual setTarget function for this" << endl);
  }
}

void SVHController::enableChannel(const SVHChannel &channel)
{
  SVHSerialPacket serial_packet(0,SVH_SET_CONTROLLER_STATE);
  SVHControllerState controller_state;
  ArrayBuilder ab(40);

  LOGGING_TRACE_C(DriverSVH, SVHController, "Enable of channel " << channel << " requested."<< endl);

  // In case no channel was enabled we need to enable the 12V dc drivers first
  if (m_enable_mask == 0)
  {
    LOGGING_TRACE_C(DriverSVH, SVHController, "Enable was called and no channel was previously activated, commands are sent individually......" << endl);
    LOGGING_TRACE_C(DriverSVH, SVHController, "Sending pwm_fault and pwm_otw...(0x001F) to reset software warnings" << endl);
    // Reset faults and overtemperature warnings saved in the controller
    controller_state.pwm_fault = 0x001F;
    controller_state.pwm_otw   = 0x001F;
    ab << controller_state;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);
    ab.reset(40);

    // Small delays seem to make communication at this point more reliable although they SHOULD NOT be necessary
    icl_core::os::usleep(2000);

    LOGGING_TRACE_C(DriverSVH, SVHController, "Enabling 12V Driver (pwm_reset and pwm_active = =0x0200)..." << endl);
    // enable +12v supply driver
    controller_state.pwm_reset = 0x0200;
    controller_state.pwm_active = 0x0200;
    ab << controller_state;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);
    ab.reset(40);

    icl_core::os::usleep(2000);

     LOGGING_TRACE_C(DriverSVH, SVHController, "Enabling pos_ctrl and cur_ctrl..." << endl);
    // enable controller
    controller_state.pos_ctrl = 0x0001;
    controller_state.cur_ctrl = 0x0001;
    ab << controller_state;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);
    ab.reset(40);

     icl_core::os::usleep(2000);

    LOGGING_TRACE_C(DriverSVH, SVHController, "...Done" << endl);
  }

  // enable actual channels (again we only accept individual channels for safety)
  if (channel >=0 && channel < eSVH_DIMENSION)
  {
    LOGGING_TRACE_C(DriverSVH, SVHController, "Enabling motor: "<< channel << endl);
    // oring all channels to create the activation mask-> high = channel active
    m_enable_mask |= (1<<channel);

    // SENDING EVERYTHING AT ONCE WILL LEAD TO "Jumping" behaviour of the controller on faster Systems ---> this has to
    // do with the initialization of the hardware controllers. If we split it in two calls we will reset them first and then activate
    // making sure that all values are initialized properly effectively preventing any jumping behaviour
    ab.reset(40);
    controller_state.pwm_fault = 0x001F;
    controller_state.pwm_otw   = 0x001F;
    controller_state.pwm_reset = (0x0200 | (m_enable_mask & 0x01FF));
    controller_state.pwm_active =(0x0200 | (m_enable_mask & 0x01FF));
    ab << controller_state;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);
    ab.reset(40);

    // WARNING: DO NOT ! REMOVE THESE DELAYS OR THE HARDWARE WILL! FREAK OUT! (see reason above)
    icl_core::os::usleep(500);

    controller_state.pos_ctrl = 0x0001;
    controller_state.cur_ctrl = 0x0001;
    ab << controller_state;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);
    ab.reset(40);

    LOGGING_DEBUG_C(DriverSVH, SVHController, "Enabled channel: " << channel << endl);
  }
  else
  {
    LOGGING_ERROR_C(DriverSVH, SVHController, "Activation request for ALL or unknown channel: "<< channel << "- ignoring request"<< endl);
  }

}

void SVHController::disableChannel(const SVHChannel& channel)
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Disable of channel " << channel << " requested."<< endl);

  if (m_serial_interface != NULL && m_serial_interface->isConnected())
  {
    // prepare general packet
    SVHSerialPacket serial_packet(0,SVH_SET_CONTROLLER_STATE);
    SVHControllerState controller_state;
    ArrayBuilder ab(40);

    // we just accept it at this point because it makes no difference in the calls
    if (channel == eSVH_ALL)
    {
      m_enable_mask = 0;
      controller_state.pwm_fault = 0x001F;
      controller_state.pwm_otw   = 0x001F;

      // default initialization to zero -> controllers are deactivated
      ab << controller_state;
      serial_packet.data = ab.array;
      m_serial_interface->sendPacket(serial_packet);

      LOGGING_DEBUG_C(DriverSVH, SVHController, "Disabled all channels"<< endl);
    }
    else if (channel >=0 && channel < eSVH_DIMENSION)
    {
      controller_state.pwm_fault = 0x001F;
      controller_state.pwm_otw   = 0x001F;
      //Disable the finger in the bitmask
      m_enable_mask &= ~(1<<channel);

      if (m_enable_mask != 0) // pos and current control stay on then
      {
        controller_state.pwm_reset  = (0x0200 | (m_enable_mask & 0x01FF));
        controller_state.pwm_active = (0x0200 | (m_enable_mask & 0x01FF));
        controller_state.pos_ctrl   = 0x0001;
        controller_state.cur_ctrl   = 0x0001;
      }

      ab << controller_state;
      serial_packet.data = ab.array;
      m_serial_interface->sendPacket(serial_packet);

      LOGGING_DEBUG_C(DriverSVH, SVHController, "Disabled channel: " << channel << endl);
    }
    else
    {
      LOGGING_WARNING_C(DriverSVH, SVHController, "Disable was requestet for unknown channel: "<< channel << "- ignoring request" << endl);
    }
  }
  else
  {
    LOGGING_ERROR_C(DriverSVH, SVHController, "Disabling Channel not possible. Serial interface is not connected!" << endl);
  }
}

void SVHController::requestControllerState()
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Requesting ControllerStatefrom Hardware"<< endl);
  SVHSerialPacket serial_packet(40,SVH_GET_CONTROLLER_STATE);
  m_serial_interface ->sendPacket(serial_packet);
}

void SVHController::requestControllerFeedback(const SVHChannel& channel)
{
  if ((channel != eSVH_ALL) && (channel >=0 && channel < eSVH_DIMENSION))
  {
    SVHSerialPacket serial_packet(40,SVH_GET_CONTROL_FEEDBACK|static_cast<uint8_t>(channel << 4));
    m_serial_interface ->sendPacket(serial_packet);

    // Disabled as it spams the output to much
    //LOGGING_TRACE_C(DriverSVH, SVHController, "Controller feedback was requested for channel: "<< channel << endl);

  }
  else if (channel == eSVH_ALL)
  {
    SVHSerialPacket serial_packet(40,SVH_GET_CONTROL_FEEDBACK_ALL);
    m_serial_interface ->sendPacket(serial_packet);

    // Disabled as it spams the output to much
    //LOGGING_TRACE_C(DriverSVH, SVHController, "Controller feedback was requested for all channels " << endl);
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Controller feedback was requestet for unknown channel: "<< channel << "- ignoring request"<< endl);
  }
}

void SVHController::requestPositionSettings(const SVHChannel& channel)
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Requesting PositionSettings from Hardware for channel: " << channel << endl);
  SVHSerialPacket serial_packet((SVH_GET_POSITION_SETTINGS| static_cast<uint8_t>(channel << 4)),40);
  m_serial_interface ->sendPacket(serial_packet);
}

void SVHController::setPositionSettings(const SVHChannel& channel,const SVHPositionSettings& position_settings)
{
  if ((channel != eSVH_ALL) && (channel >=0 && channel < eSVH_DIMENSION))
  {
    SVHSerialPacket serial_packet(0,SVH_SET_POSITION_SETTINGS|static_cast<uint8_t>(channel << 4));
    ArrayBuilder ab;
    ab << position_settings;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);

    // Save already in case we dont get immediate response
    m_position_settings[channel] = position_settings;

    LOGGING_DEBUG_C(DriverSVH, SVHController, "Position controller settings where send to change channel: "<< channel << " : ");
    LOGGING_DEBUG_C(DriverSVH, SVHController,  "wmn " << position_settings.wmn << " " << "wmx " << position_settings.wmx << " " << "dwmx "<< position_settings.dwmx << " "
                                            << "ky "  << position_settings.ky  << " " << "dt "  << position_settings.dt  << " " << "imn " << position_settings.imn << " "
                                            << "imx " << position_settings.imx << " " << "kp "  << position_settings.kp  << " " << "ki "  << position_settings.ki  << " "
                                            << "kd "  << position_settings.kd << " " << endl );

  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Position controller settings where given for unknown channel: "<< channel << "- ignoring request"<< endl);
  }
}

void SVHController::requestCurrentSettings(const SVHChannel& channel)
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Requesting CurrentSettings for channel: " << channel << endl);

  if ((channel != eSVH_ALL) && (channel >=0 && channel < eSVH_DIMENSION))
  {
    SVHSerialPacket serial_packet(40,(SVH_GET_CURRENT_SETTINGS|static_cast<uint8_t>(channel << 4)));
    m_serial_interface ->sendPacket(serial_packet);
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Get Current Settings can only be requested with a specific channel, ALL or unknown channel:" << channel << "was selected " << endl);
  }
}

void SVHController::setCurrentSettings(const SVHChannel& channel,const SVHCurrentSettings& current_settings)
{
  if ((channel != eSVH_ALL) && (channel >=0 && channel < eSVH_DIMENSION))
  {
    SVHSerialPacket serial_packet(0,SVH_SET_CURRENT_SETTINGS|static_cast<uint8_t>(channel << 4));
    ArrayBuilder ab;
    ab << current_settings;
    serial_packet.data = ab.array;
    m_serial_interface ->sendPacket(serial_packet);

    // Save already in case we dont get immediate response
    m_current_settings[channel] = current_settings;

    LOGGING_DEBUG_C(DriverSVH, SVHController, "Current controller settings where send to change channel: "<< channel << " : ");
    LOGGING_DEBUG_C(DriverSVH, SVHController, "wmn "<< current_settings.wmn << " " << "wmx "<< current_settings.wmx << " " << "ky " << current_settings.ky  << " "
                                           << "dt " << current_settings.dt  << " " << "imn "<< current_settings.imn << " " << "imx "<< current_settings.imx << " "
                                           << "kp " << current_settings.kp  << " " << "ki " << current_settings.ki  << " " << "umn "<< current_settings.umn << " "
                                           << "umx "<< current_settings.umx << endl);

  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Current controller settings where given for unknown channel: "<< channel << "- ignoring request"<< endl);
  }
}

void SVHController::requestEncoderValues()
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Requesting EncoderValues from hardware"<< endl);
  SVHSerialPacket serial_packet(40,SVH_GET_ENCODER_VALUES);
  m_serial_interface ->sendPacket(serial_packet);
}

void SVHController::setEncoderValues(const SVHEncoderSettings &encoder_settings)
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Setting new Encoder values : ");
  for (size_t i = 0; i < encoder_settings.scalings.size(); i++)
  {
    // log stream unfortunately does not support the standard to stream operators yet
    LOGGING_TRACE_C(DriverSVH, SVHController, "[" << (int)i << "] " << ": " <<encoder_settings.scalings[i] << " " );
  }
  LOGGING_TRACE_C(DriverSVH, SVHController, endl);

  SVHSerialPacket serial_packet(0,SVH_SET_ENCODER_VALUES);
  ArrayBuilder ab;
  ab << encoder_settings;
  serial_packet.data = ab.array;
  m_serial_interface ->sendPacket(serial_packet);

  // Save already in case we dont get imediate response
  m_encoder_settings = encoder_settings;
}

void SVHController::requestFirmwareInfo()
{
  LOGGING_TRACE_C(DriverSVH, SVHController, "Requesting firmware Information from hardware" << endl);

  SVHSerialPacket serial_packet(40,SVH_GET_FIRMWARE_INFO);
  m_serial_interface->sendPacket(serial_packet);
}

void SVHController::receivedPacketCallback(const SVHSerialPacket& packet, unsigned int packet_count)
{
  // Extract Channel
  uint8_t channel = (packet.address >> 4 ) & 0x0F;
  // Prepare Data for conversion
  ArrayBuilder ab;
  ab.appendWithoutConversion(packet.data);

  SVHControllerFeedbackAllChannels feedback_all;

  m_received_package_count = packet_count;

  // Packet meaning is encoded in the lower nibble of the adress byte
  switch (packet.address & 0x0F)
  {
    case SVH_GET_CONTROL_FEEDBACK:
    case SVH_SET_CONTROL_COMMAND:
      if (channel >=0 && channel < eSVH_DIMENSION)
      {
        //std::cout << "Recieved: Controllerfeedback RAW Data: " << ab;
        ab >> m_controller_feedback[channel];
        // Disabled as this is spamming the output to much
        //LOGGING_TRACE_C(DriverSVH, SVHController, "Received a Control Feedback/Control Command packet for channel "<< channel << " Position: "<< (int)m_controller_feedback[channel].position  << " Current: "<< (int)m_controller_feedback[channel].current << endl);
      }
      else
      {
        LOGGING_ERROR_C(DriverSVH, SVHController, "Received a Control Feedback/Control Command packet for ILLEGAL channel "<< channel << "- packet ignored!" << endl);
      }
      break;
    case SVH_GET_CONTROL_FEEDBACK_ALL:
    case SVH_SET_CONTROL_COMMAND_ALL:
      // We cannot just read them all into the vector (which would have been nice) because the feedback of all channels is structured
      // different from the feedback of one channel. So the SVHControllerFeedbackAllChannels is used as an intermediary ( handles the deserialization)
      ab >> feedback_all;
      m_controller_feedback = feedback_all.feedbacks;
      // Disabled as this is spannimg the output to much
      //LOGGING_TRACE_C(DriverSVH, SVHController, "Received a Control Feedback/Control Command packet for channel all channels "<<  endl);
      break;
    case SVH_GET_POSITION_SETTINGS:
    case SVH_SET_POSITION_SETTINGS:
      if (channel >=0 && channel < eSVH_DIMENSION)
      {
        //std::cout << "Recieved: Postitionsettings RAW Data: " << ab; // for really intensive debugging
        ab >> m_position_settings[channel];
        LOGGING_TRACE_C(DriverSVH, SVHController, "Received a get/set position setting packet for channel "<< channel  << endl);
        LOGGING_TRACE_C(DriverSVH, SVHController, "wmn " << m_position_settings[channel].wmn << " "<< "wmx " << m_position_settings[channel].wmx << " "<< "dwmx "<< m_position_settings[channel].dwmx << " "<< "ky "  << m_position_settings[channel].ky  << " "<< "dt "  << m_position_settings[channel].dt  << " "<< "imn " << m_position_settings[channel].imn << " "<< "imx " << m_position_settings[channel].imx << " " << "kp "  << m_position_settings[channel].kp  << " " << "ki "  << m_position_settings[channel].ki  << " " << "kd "  << m_position_settings[channel].kd  << endl);

      }
      else
      {
        LOGGING_ERROR_C(DriverSVH, SVHController, "Received a get/set position setting packet for ILLEGAL channel "<< channel << "- packet ignored!" << endl);
      }
      break;
    case SVH_GET_CURRENT_SETTINGS:
    case SVH_SET_CURRENT_SETTINGS:
      if (channel >=0 && channel < eSVH_DIMENSION)
      {
        //std::cout << "Recieved: Current Settings RAW Data: " << ab; // for really intensive debugging
        ab >> m_current_settings[channel];
        LOGGING_TRACE_C(DriverSVH, SVHController, "Received a get/set current setting packet for channel "<< channel << endl);
        LOGGING_TRACE_C(DriverSVH, SVHController, "wmn "<< m_current_settings[channel].wmn << " " << "wmx "<< m_current_settings[channel].wmx << " " << "ky " << m_current_settings[channel].ky  << " " << "dt " << m_current_settings[channel].dt  << " " << "imn "<< m_current_settings[channel].imn << " " << "imx "<< m_current_settings[channel].imx << " "                   << "kp " << m_current_settings[channel].kp  << " " << "ki " << m_current_settings[channel].ki  << " " << "umn "<< m_current_settings[channel].umn << " " << "umx "<< m_current_settings[channel].umx << " "<< endl);
      }
      else
      {
        LOGGING_ERROR_C(DriverSVH, SVHController, "Received a get/set current setting packet for ILLEGAL channel "<< channel << "- packet ignored!" << endl);
      }
      break;
    case SVH_GET_CONTROLLER_STATE:
    case SVH_SET_CONTROLLER_STATE:
        //std::cout << "Recieved: Controller State RAW Data: " << ab; // for really intensive debugging
        ab >> m_controller_state;
        //std::cout << "Received controllerState interpreded data: "<< m_controller_state << std::endl; // for really intensive debugging
        LOGGING_TRACE_C(DriverSVH, SVHController, "Received a get/set controler state packet " << endl);
        LOGGING_TRACE_C(DriverSVH, SVHController, "Controllerstate (NO HEX):" << "pwm_fault " << "0x" << static_cast<int>(m_controller_state.pwm_fault) << " " << "pwm_otw " << "0x" << static_cast<int>(m_controller_state.pwm_otw) << " "  << "pwm_reset " << "0x" << static_cast<int>(m_controller_state.pwm_reset) << " " << "pwm_active " << "0x" << static_cast<int>(m_controller_state.pwm_active) << " " << "pos_ctr " << "0x" <<  static_cast<int>(m_controller_state.pos_ctrl) << " " << "cur_ctrl " << "0x" << static_cast<int>(m_controller_state.cur_ctrl) << endl);
      break;
    case SVH_GET_ENCODER_VALUES:
    case SVH_SET_ENCODER_VALUES:
        LOGGING_TRACE_C(DriverSVH, SVHController, "Received a get/set encoder settings packet " << endl);
        ab >> m_encoder_settings;
      break;
    case SVH_GET_FIRMWARE_INFO:
        //std::cout << "Recieved: Firmware Settings RAW Data: " << ab; // for really intensive debugging
        ab >> m_firmware_info;
        LOGGING_INFO(DriverSVH, "Hardware is using the following Firmware: " );
        LOGGING_INFO(DriverSVH, m_firmware_info.svh  << " Version: " << m_firmware_info.version_major << "." << m_firmware_info.version_minor << " : " << m_firmware_info.text << endl);
      break;
    default:
        LOGGING_ERROR_C(DriverSVH, SVHController, "Received a Packet with unknown address: "<< (packet.address & 0x0F) << " - ignoring packet" << endl);
      break;
  }

}

bool SVHController::getControllerFeedback(const SVHChannel &channel,SVHControllerFeedback& controller_feedback)
{
  if(channel >= 0 && static_cast<uint8_t>(channel) < m_controller_feedback.size())
  {
    controller_feedback = m_controller_feedback[channel];
    return true;
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "GetFeedback was requested for unknown channel: "<< channel<< "- ignoring request" << endl);
    return false;
  }
}

void SVHController::getControllerFeedbackAllChannels(SVHControllerFeedbackAllChannels& controller_feedback)
{
  controller_feedback.feedbacks = m_controller_feedback;
}

bool SVHController::getPositionSettings(const SVHChannel &channel, SVHPositionSettings &position_settings)
{
  if(channel >= 0 && static_cast<uint8_t>(channel) < m_position_settings.size())
  {
    position_settings = m_position_settings[channel];
    return true;
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "GetPositionSettings was requested for unknown channel: "<< channel<< "- ignoring request" << endl);
    return false;
  }
}

bool SVHController::getCurrentSettings(const SVHChannel &channel, SVHCurrentSettings &current_settings)
{
  if(channel >= 0 && static_cast<uint8_t>(channel) < m_current_settings.size())
  {
    current_settings = m_current_settings[channel];
    return true;
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "GetCurrentSettings was requested for unknown channel: "<< channel<< "- ignoring request" << endl);
    return false;
  }
}

SVHFirmwareInfo SVHController::getFirmwareInfo()
{
  return m_firmware_info;
}

void SVHController::resetPackageCounts()
{
  m_received_package_count = 0;
  m_serial_interface->resetTransmitPackageCount();
  LOGGING_TRACE_C(DriverSVH, SVHController, "Received package count resetted" << endl);
}

unsigned int SVHController::getSentPackageCount()
{
  if (m_serial_interface != NULL)
  {
    return m_serial_interface->transmittedPacketCount();
  }
  else
  {
    LOGGING_WARNING_C(DriverSVH, SVHController, "Request for transmit packet count could not be answered as the device is not connected - ignoring request" << endl);
    return 0;
  }
}

unsigned int SVHController::getReceivedPackageCount()
{
  return m_received_package_count;
}

bool SVHController::isEnabled(const SVHChannel &channel)
{
  return ((1 << channel & m_enable_mask) > 0);
}




}
