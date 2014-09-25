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
 * \date    2014-07-16
 *
 * This file contains the ControllerStateFeedback data structure that is used to
 * send and receive feedback of the current state of the hardware controller
 * this includes activation of the position as well as the current controllers
 * overtemperature and fault warnings and activation of power supplies
 */
//----------------------------------------------------------------------
#ifndef SVHCONTROLLERSTATE_H
#define SVHCONTROLLERSTATE_H

namespace driver_svh {

/*!
 * \brief The SVHControllerState indicates the current state of the MeCoVis controller IC which is used in the SVH
 */
struct SVHControllerState
{
  //! Fault indication of the controllers (0x001F -> high bit to reset software state. 0x00 -> low bit to reset hardware)
  //! \note The Faults are not yet mapped to channels directly so the information should only be treated as a fault/no fault
  uint16_t pwm_fault;
  //! Over Temperature Warning of the controllers (0x001F -> high bit to reset software state. 0x00 -> low bit to reset hardware)
  //! \note The Faults are not yet mapped to channels directly so the information should only be treated as a fault/no fault
  uint16_t pwm_otw;
  //! Bitmask for low-active resets of the channels (0-8). Channel 9 (0x0200) is a special channel that activates a 12VDV-DV converter for the small motors.
  uint16_t pwm_reset;
  //! Currently unused but should be called the same way as pwm_reset is
  //! \note This flag is currently unused by the hardware but should be handled exactly the same as pwm reset at the moment
  uint16_t pwm_active;
  //! Enable/Disable of position controller (0x0001 to Activate )
  uint16_t pos_ctrl;
  //! Enable/Disbale of current controller (0x0001 to Activate)
  uint16_t cur_ctrl;

  //! Default constructor with zero values for everything
  SVHControllerState(uint16_t _pwm_fault =0,uint16_t _pwm_otw = 0 , uint16_t _pwm_reset = 0 ,uint16_t _pwm_active = 0 ,uint16_t _pos_ctrl =0, uint16_t _cur_ctrl =0):
  pwm_fault(_pwm_fault),
  pwm_otw(_pwm_otw),
  pwm_reset(_pwm_reset),
  pwm_active(_pwm_active),
  pos_ctrl(_pos_ctrl),
  cur_ctrl(_cur_ctrl)
  {  }

  //! Compares two SVHControllerState objects.
  bool operator == (const SVHControllerState& other) const
  {
    return
      (pwm_fault == other.pwm_fault
       && pwm_otw == other.pwm_otw
       && pwm_reset == other.pwm_reset
       && pwm_active == other.pwm_active
       && pos_ctrl == other.pos_ctrl
       && cur_ctrl == other.cur_ctrl);
  }
};

//! overload stream operator to easily serialize controller state data
inline icl_comm::ArrayBuilder& operator << (icl_comm::ArrayBuilder& ab, const SVHControllerState& data)
{
  ab << data.pwm_fault
     << data.pwm_otw
     << data.pwm_reset
     << data.pwm_active
     << data.pos_ctrl
     << data.cur_ctrl;
  return ab;
}

//! overload stream operator to easily serialize controller state data
inline icl_comm::ArrayBuilder& operator >> (icl_comm::ArrayBuilder& ab, SVHControllerState& data)
{
  ab >> data.pwm_fault
     >> data.pwm_otw
     >> data.pwm_reset
     >> data.pwm_active
     >> data.pos_ctrl
     >> data.cur_ctrl;

  return ab;
}

//! Output Stream operator to easily output controller state data
inline std::ostream& operator << (std::ostream& o, const SVHControllerState& cs)
{
  o << std::setw(4) << std::setfill('0') << std::hex
    << "pwm_fault " << "0x" << static_cast<int>(cs.pwm_fault) << " "
    << "pwm_otw " << "0x" << static_cast<int>(cs.pwm_otw) << " "
    << "pwm_reset " << "0x" << static_cast<int>(cs.pwm_reset) << " "
    << "pwm_active " << "0x" << static_cast<int>(cs.pwm_active) << " "
    << "pos_ctr " << "0x" <<  static_cast<int>(cs.pos_ctrl) << " "
    << "cur_ctrl " << "0x" << static_cast<int>(cs.cur_ctrl) << " "
    << std::endl;
  // Reset Output stream to decimal output .. otherwise it may confuse people
  std::cout << std::dec ;
  return o;
}

}


#endif // SVHCONTROLLERSTATE_H
