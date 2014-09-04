// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars pfotzer
 * \date    2014-01-30
 *
 * This file contains the Finger Manager
 * that is managing the Schunk five finger hand on a high level.
 * The fingermanager is responsible to filter all calls and only make
 * allowed calls to the controller. The fingermanager is also responsible
 * for storing any kind of Konfiguration (like current controller settings).
 * The fingermanager is also responsible to poll the controller for continious data (if wanted)
 */
//----------------------------------------------------------------------
#ifndef DRIVER_SVH_SVH_FINGER_MANAGER_H_INCLUDED
#define DRIVER_SVH_SVH_FINGER_MANAGER_H_INCLUDED

#include <driver_svh/ImportExport.h>
#include <driver_svh/SVHController.h>
#include <driver_svh/SVHFeedbackPollingThread.h>

#include <boost/shared_ptr.hpp>

#ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
#include <icl_comm_websocket/WsBroadcaster.h>
#else
// Forward Deklaration of the WsBroadcaster
// This is not needed for normal driver operation
// but might be added later. To keep the interface the same
// a forward declaration becomes necessary
namespace icl_comm{
namespace websocket{
  class WsBroadcaster;
}}// NS end
#endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_




namespace driver_svh {

/*! This class manages controller parameters and the finger reset.
 */
class DRIVER_SVH_IMPORT_EXPORT SVHFingerManager
{
public:

  /*!
   * \brief The MovementState enum indicated the overall state of the hand. Currently only used for updating the status websocket
   */
  enum MovementState
  {
    eST_DEACTIVATED,
    eST_RESETTING,
    eST_RESETTED,
    eST_ENABLED,
    eST_PARTIALLY_ENABLED,
    eST_FAULT,
    eST_DIMENSION
  };

  /*! Constructs a finger manager for the SCHUNK five finger hand.
   * \param autostart if set to true, the driver will immediately connect to the hardware and try to reset all fingers
   * \param dev_name the dev to use for autostart. Default is /dev/ttyUSB0
   */
  SVHFingerManager(const bool &autostart = false, const std::vector<bool> &disable_mask = std::vector<bool>(9,false), const std::string &dev_name = "/dev/ttyUSB0");

  virtual ~SVHFingerManager();

 /*!
  *  \brief Open connection to SCHUNK five finger hand. Wait until expected return packages are received.
  *  \param dev_name
  * \return
  */
  bool connect(const std::string &dev_name);

  //!
  //! \brief disconnect SCHUNK five finger hand
  //!
  void disconnect();

  //!
  //! \brief returns connected state of finger manager
  //! \return bool
  //!
  bool isConnected() { return m_connected; }

  //!
  //! \brief reset function for channel
  //! \param channel
  //! \return
  //!
  bool resetChannel(const SVHChannel &channel);

  //!
  //! \brief enable controller of channel
  //! \param channel
  //! \return
  //!
  bool enableChannel(const SVHChannel &channel);

  //!
  //! \brief disable controller of channel
  //! \param channel
  //! \return
  //!
  void disableChannel(const SVHChannel &channel);

  //!
  //! \brief sends request controller feedback packet for all channels
  //! \return
  //!
  bool requestControllerFeedbackAllChannels();

  //!
  //! \brief send request controller feedback paket
  //! \param channel
  //! \return
  //!
  bool requestControllerFeedback(const SVHChannel &channel);

  //!
  //! \brief returns position value of channel
  //! \param channel
  //! \param position
  //! \return bool
  //!
  bool getPosition(const SVHChannel &channel, double &position);

  //!
  //! \brief returns current value of channel
  //! \param channel
  //! \param current
  //! \return bool
  //!
  bool getCurrent(const SVHChannel &channel, double &current);

  //!
  //! \brief returns actual current controller settings of channel
  //! \param channel
  //! \param current_settings
  //! \return
  //!
  bool getCurrentControllerParams(const SVHChannel &channel, SVHCurrentSettings &current_settings);

  //!
  //! \brief returns actual position controller settings of channel
  //! \param channel
  //! \param position_settings
  //! \return
  //!
  bool getPositionControllerParams(const SVHChannel &channel, SVHPositionSettings &position_settings);

  //!
  //! \brief set all target positions at once
  //! \param positions
  //! \return
  //!
  bool setAllTargetPositions(const std::vector<double>& positions);

  //!
  //! \brief set target position of a channel
  //! \param channel
  //! \param position
  //! \param current
  //! \return
  //!
  bool setTargetPosition(const SVHChannel &channel, double position, double current);

  //!
  //! \brief overwrite current parameters
  //! \param channel
  //! \param current_settings
  //! \return
  //!
  bool setCurrentControllerParams(const SVHChannel &channel, const SVHCurrentSettings &current_settings);

  //!
  //! \brief overwrite position parameters
  //! \param channel
  //! \param position_settings
  //! \return
  //!
  bool setPositionControllerParams(const SVHChannel &channel, const SVHPositionSettings &position_settings);

  //!
  //! \brief returns true, if current channel has been enabled
  //! \param channel
  //! \return
  //!
  bool isEnabled(const SVHChannel &channel);

  //!
  //! \brief returns true, if current channel has been resetted
  //! \param channel
  //! \return
  //!
  bool isHomed(const SVHChannel &channel);

  /*!
   * \brief setMovementState Updates the movement state of the overll hand indicating the overall status
   * \param state current movement state
   * \note this is only used for monitoring purposes at the moment, driverwise there is no need to call it but it is used by web frontends
   */
  void setMovementState(const MovementState &state);


  //! This is a debuging function. Should not be called by users
  void requestControllerState();


  // These 3 functions could be private but where made public for printing and debug purposes. As there is no harm to it it should not be a problem

  //! \brief get default parameters for position settings DURING RESET
  std::vector<SVHPositionSettings> getPositionSettingsDefaultResetParameters();

  //! \brief get default parameters for current settings
  std::vector<SVHCurrentSettings> getCurrentSettingsDefaultParameters();

  //! \brief get default parameters for position settings
  std::vector<SVHPositionSettings> getPositionSettingsDefaultParameters();



  #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
  /*!
   * \brief updateWebSocket Will gathe the current state of the hand and send it out via websocket
   * \note this function will NOT update everything as it would be to much overhead to ask every single time if a finger is enabled or not. Things
   *       that happen only sometimes will be updated in the corresponding functions (enable, diable, reset and so on)
   *       this function is meant to be used for the periodically changing states
   */
  void updateWebSocket();
  #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_

// ----------------------------------------------------------------------
// ---- private functions and varaibles
// ----------------------------------------------------------------------

private:

  //! \brief Websocket handle for updating diagnostic backend (OPTIONAL)
  boost::shared_ptr<icl_comm::websocket::WsBroadcaster> m_ws_broadcaster;

  //! \brief pointer to svh controller
  SVHController *m_controller;

  //! \brief pointer to svh controller
  SVHFeedbackPollingThread *m_feedback_thread;

  //! \brief holds the connected state
  bool m_connected;

  //! \brief vector storing reset flags for each finger
  int8_t m_homing_timeout;

  //! data sctructure for home positions
  struct
  {
    int   direction;         // +1 or -1 : home in positive or negative direction
    float minimumOffset;     // offset from home position to minimum (soft limit)
    float maximumOffset;     // offset from home position to maximum (soft limit)
    float idlePosition;      // position to go to after intialization
  } typedef HomeSettings;

  //! \brief home position default settings vector for each channel
  std::vector<HomeSettings> m_home_settings;

  //! \brief position conversion factor (ticks to RAD) for each channel
  std::vector<double> m_ticks2rad;

  //! \brief min position vector for each channel
  std::vector<int32_t> m_position_min;

  //! \brief max position vector for each channel
  std::vector<int32_t> m_position_max;

  //! \brief home position after complete reset of each channel
  std::vector<int32_t> m_position_home;

  //! \brief vector storing reset flags for each channel
  std::vector<bool> m_is_homed;

  //! vector storing information if a finger is enabled. In Case it is all request for it will be granted but not executed on hardware
  std::vector<bool> m_is_switched_off;

  //! Overall movement State to indicate what the hand is doing at the moment
  MovementState m_movement_state;

  //! \brief vector storing the reset order of the channels
  std::vector<SVHChannel> m_reset_order;

  /*!
    * \brief Vector containing factors for the currents at reset.
    * Vector containing factors for the currents at reset.
    * A hard stop is found if the maxCurrent (first 2 CurrentSettingsValues) x the reset factor was reached. 0.75 by default
    * Beware. Setting this value very high might result in damage to the motors during reset.
    */
  std::vector<double> m_reset_current_factor;



  //! \brief set default parameters for home position
  void setHomePositionDefaultParameters();



  //!
  //! \brief Converts joint positions of a specific channel from RAD to ticks
  //! \param channel
  //! \param position
  //! \return
  //!
  int32_t convertRad2Ticks(const SVHChannel &channel, double position);

  //!
  //! \brief Check bounds of target positions
  //! \param channel
  //! \param target_position
  //! \return
  //!
  bool isInsideBounds(const SVHChannel &channel, const int32_t &target_position);

  //!
  //! \brief readParametersFromConfigFile
  //! \return
  //!
  bool readParametersFromConfigFile();


  // DEBUG
  SVHControllerFeedback debug_feedback;


};

}

#endif
