// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-


// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of FZIs ic_workspace.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2014-5-22
 *
 */
//----------------------------------------------------------------------

#include "LWA4PState.h"

#include <string>
#include <json/json.h>
#include <json/writer.h>
#include <boost/lexical_cast.hpp>

namespace icl_comm {
namespace websocket {

LWA4PState::LWA4PState(): RobotState(6),
  m_movement_state(eST_DEACTIVATED)
{
}


std::string LWA4PState::getStateJSON()
{
  Json::Value message;
  Json::FastWriter json_writer;

  message["type"] = "arm";
  switch (m_movement_state)
  {
  case eST_DEACTIVATED:
      message["state"] = "deactivated";
    break;
  case eST_CONTROLLED:
      message["state"] = "controlled";
    break;
  case eST_FAULT: message["state"] = "fault";
    break;
  case eST_CHAIN_BUILT_UP:
      message["state"] = "chain built up";
    break;
  case eST_IPM_MODE:
      message["state"] = "ipm mode";
    break;
  case eST_ERROR_ACKED: message["state"] = "error cleared";
    break;
  case eST_COMMUTATION_FOUND:
      message["state"] = "commutation found";
    break;
  case eST_PSEUDE_ABSOLUT_CALIBRATED:
      message["state"] = "pseudo absolut calibrated";
    break;
  case eST_JITTER_SYNCED:
      message["state"] = "jitter synced";
    break;
  case eST_SWITCH_OF_POSE_DIFFERS:
      message["state"] = "switch of pose differs";
    break;
  case eST_SWITCH_OF_POSE_VERIFIED:
      message["state"] = "switch of pose verified";
    break;
  case eST_READY:
      message["state"] = "ready";
    break;
  case eST_ENABLED:
      message["state"] = "enabled";
    break;
  case eST_ABSOLUTE_POSITION_LOST:
    message["state"] = "ERROR: Absolute calibration was lost";
    break;
  default:
    message["state"] = "unknown";
  };

  for (size_t i = 0; i < m_num_axes; ++i)
  {
    std::string joint = "joint"+boost::lexical_cast<std::string>(i);
    message["position"][joint] = m_joint_positions[i];
    message["speed"][joint] = m_joint_velocities[i];
    message["leds"][joint] = m_joint_errors[i];
    message["enabled"][joint] = m_joints_enabled[i];
    message["homed"][joint] = m_joints_homed[i];
  }


  return json_writer.write(message);
}

void LWA4PState::setMovementState(const int &movement_state)
{
  m_movement_state = static_cast<MovementState>(movement_state);
}


}} // end of NS
