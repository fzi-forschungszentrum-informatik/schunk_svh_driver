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
 * \author  Georg Heppner
 * \date    2014-01-30
 *
 * This file contains the SVH controler, the heart of the driver.
 * It is responsible to manage all logical decissions regarding the hardware
 * on a low level. It knows what packet index is used for which function
 * and holds all the data objects that can be queried externally.
 * The controller should not be queried by outside calls directly as it asumes
 * the calls to be non maleformed or contain out of bounds acces as this is handled
 * by the finger manager.Only minimal safeguards are in place.
 * Also note that the calls on this level should be made
 * channel wise. The iteration of channels is done in the finger controller.
 *
 * Request and Get principle: As the communication with the hand had some issues with the bandwith
 * there are two types of function calls. The request functions tell the driver to actually request the data
 * from the hardware. The get functions just get the last received value from the controller without actually
 * querrying the hardware. This might be changed in further releases.
 */
//----------------------------------------------------------------------
#ifndef DRIVER_SVH_SVH_CONTROLLER_H_INCLUDED
#define DRIVER_SVH_SVH_CONTROLLER_H_INCLUDED

#include <driver_svh/ImportExport.h>
#include <driver_svh/SVHSerialInterface.h>
#include <driver_svh/SVHReceiveThread.h>
#include <driver_svh/SVHControlCommand.h>
#include <driver_svh/SVHControllerFeedback.h>
#include <driver_svh/SVHCurrentSettings.h>
#include <driver_svh/SVHFirmwareInfo.h>
#include <driver_svh/SVHPositionSettings.h>
#include <driver_svh/SVHControllerState.h>
#include <driver_svh/SVHEncoderSettings.h>

namespace driver_svh {

//! Channel indicates which motor to use in command calls. WARNING: DO NOT CHANGE THE ORDER OF THESE as it represents the hardware mapping
enum{
  eSVH_ALL = -1,   // this should be used with care as not all functions support it yet
  eSVH_THUMB_FLEXION = 0,
  eSVH_THUMB_OPPOSITION, // wrist
  eSVH_INDEX_FINGER_DISTAL,
  eSVH_INDEX_FINGER_PROXIMAL,
  eSVH_MIDDLE_FINGER_DISTAL,
  eSVH_MIDDLE_FINGER_PROXIMAL,
  eSVH_RING_FINGER,
  eSVH_PINKY,
  eSVH_FINGER_SPREAD,
  eSVH_DIMENSION //9
} typedef SVHChannel;

/*!
 * \brief This class controls the the SCHUNK five finger hand.
 *
 * The controller manages all calls to the hardware and receives every feedback.
 * All data is interpreted and stored in the apropriate objects that can be queried by others.
 * \note Be carefull what you change in here as it interfaces directly with the hardware
 */
class DRIVER_SVH_IMPORT_EXPORT SVHController
{
public:
  //!Constructs a controller class for the SCHUNK five finger hand.
  SVHController();

  /*! SCHUNK five finger hand destructor
   *  Destructor, disable the serial device and shut down hand as far as possible
   */
  ~SVHController();

  /*!
   * \brief Open serial device connection
   * \param dev_name System handle (filename in linux) to the device
   * \return true if connect was successfull
   */
  bool connect(const std::string &dev_name);

  //! disconnect serial device
  void disconnect();

  /*!
   * \brief Set new position target for finger index
   * \param channel Motorchanel to set the target for
   * \param position Target position for the channel given in encoder Ticks
   */
  void setControllerTarget(const SVHChannel& channel, const int32_t &position);

  /*!
   * \brief Setting new position controller target for all fingers
   * \param positions Target positions for all fingers, Only the first nine values will be evaluated
   */
  void setControllerTargetAllChannels(const std::vector<int32_t>& positions);


  // Access functions
  /*!
   *  \brief Enable one or all motor channels
   *  \param channel Motor to activate
   */
  void enableChannel(const SVHChannel& channel);

  /*!
   *  \brief Disable one or all motor channels
   *  \param channel Motor to deactivate
   */
  void disableChannel(const SVHChannel& channel);

  //! Request current controller state (mainly usefull for debug purposes)
  void requestControllerState();

  /*!
   *  \brief request feedback (position and current) to a specific channel
   *  \param channel Motorchannel the feedback should be provided for
   */
  void requestControllerFeedback(const SVHChannel& channel);

  /*!
   * \brief request the settings of the position controller for a specific channel
   * \param channel Motor to request the settings for
   */
  void requestPositionSettings(const SVHChannel& channel);

  /*!
   * \brief activate a new set of position controller settings for a specific channel
   * \param channel Motor the new position controller settings will be applied to
   * \param position_settings new settings of the position controller
   */
  void setPositionSettings(const SVHChannel& channel,const SVHPositionSettings& position_settings);

  /*!
   * \brief request the settings of the current controller for a specific channel
   * \param channel Motor to request the settings for
   */
  void requestCurrentSettings(const SVHChannel& channel);

  /*!
   * \brief activate a new set of current controller settings for a specific channel
   * \param channel Motor the new current controller settings will be applied to
   * \param current_settings new settings of the current controller
   */
  void setCurrentSettings(const SVHChannel& channel,const SVHCurrentSettings& current_settings);

  /*!
   * \brief read out the mutipliers for the encoders from the hardware
   */
  void requestEncoderValues();

  /*!
   * \brief sends a new set of encodervalues to the hardware
   * \param encoder_settings to set (prescalers)
   */
  void setEncoderValues(const SVHEncoderSettings& encoder_settings);


  /*!
   * \brief request a transmission of formware information
   */
  void requestFirmwareInfo();

  /*!
   * \brief callback function for interpretation of packages
   * \param packet SerialPacket containing the raw data, integrity should have been checked by SerialInterface
   * \param packet_count count of received packets
   */
  void receivedPacketCallback(const SVHSerialPacket& packet, unsigned int packet_count);

  /*!
   * \brief request the latest stored controllerfeedback (current, position) from the controller.
   * \param channel Motor to get the latest feedback to
   * \param ControllerFeedback (current, encoder position) of the specified channel
   * \return true if the feedback could be read, false otherwise
   *
   * Controllerfeedback (crurrent,channel) is stored/updated in the controller once it is send by the hardware.
   * This is the case once a controlCommand has been send or the feedback has
   * specifically been requested by using the getControllerFeedback() function
   *
   */
   bool getControllerFeedback(const SVHChannel &channel,SVHControllerFeedback& controller_feedback);

   /*!
    * \brief request the latest stored positionsettings from the controller
    * \param channel Motor to get the positionsettings for
    * \param position_settings position settings to be returned
    * \return true if the request was succesfull false otherwise
    */
   bool getPositionSettings(const SVHChannel &channel,SVHPositionSettings& position_settings);

   /*!
    * \brief request the latest stored currentsettings from the controller
    * \param channel Motor to get the currentsettings for
    * \param position_settings current settings to be returned
    * \return true if the request was succesfull false otherwise
    */
   bool getCurrentSettings(const SVHChannel &channel,SVHCurrentSettings& current_settings);

   /*!
    * \brief get the latest stored Firmware information from the controller (NOT THE HARDWARE)
    * \return the Firmware information
    */
   SVHFirmwareInfo getFirmwareInfo();

   /*!
    * \brief requests the number of sent packages. Request ist transferred to the serial interface that knows about this count
    * \return number of packages correctly sent
    */
   unsigned int getSentPackageCount();

   /*!
    * \brief request the number of correctly received packages. This number is refreshed every time the serialinterace calls the receivedPacket callback
    * \return number of packages correctly received
    */
   unsigned int getReceivedPackageCount();

   /*!
    * \brief resetPackageCounts sets the sent and reveived package counts to zero
    */
   void resetPackageCounts();

   /*!
    * \brief Check if a channel was enabled
    * \param channel to check
    * \return True if an enable has been send to the hardware
    */
   bool isEnabled(const SVHChannel &channel);

   //! Description values to get the corresponding string value to a channel enum
   static const char * m_channel_description[];

   //! Get all currently available controllerfeedbacks
   void getControllerFeedbackAllChannels(SVHControllerFeedbackAllChannels &controller_feedback);
private:

  // Data Structures for holding configurations and feedback of the Controller

  //! vector of current controller parameters for each finger
  std::vector<SVHCurrentSettings> m_current_settings;

  //! vector of position controller parameters for each finger
  std::vector<SVHPositionSettings> m_position_settings;

  //! ControllerFeedback indicates current position and current per finger
  std::vector<SVHControllerFeedback> m_controller_feedback;

  //! Currently active controllerstate on the HW Controller (indicates if PWM active etc.)
  SVHControllerState m_controller_state;

  //! Currently active encoder settings
  SVHEncoderSettings m_encoder_settings;

  //! Latest firmware info
  SVHFirmwareInfo m_firmware_info;

  // Hardware control

  //! Serial interface for transmission and reveibing of data packets
  SVHSerialInterface * m_serial_interface;

  //! Bitmask to tell which fingers are enabled
  uint16_t m_enable_mask;

  //! store how many packages where actually received. Updated every time the receivepacket callback is called
  unsigned int m_received_package_count;




};

}

#endif
