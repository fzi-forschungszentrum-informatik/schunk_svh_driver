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

#include "SVHState.h"

#include <json/json.h>
#include <json/writer.h>
#include <boost/lexical_cast.hpp>

namespace icl_comm {
namespace websocket {

//! Description for enum matching:
const std::string SVHState::m_joint_descriptions[]= {
  "thumb_flexion",
  "thumb_opposition", // wrist
  "index_distal",
  "index_proximal",
  "middle_distal",
  "middle_proximal",
  "ring",
  "pinky",
  "spread"
};


SVHState::SVHState():RobotState(9),
  m_movement_state(eST_DEACTIVATED)
{
}


std::string SVHState::getStateJSON()
{
  Json::Value message;
  Json::FastWriter json_writer;

  message["type"] = "hand";
  switch (m_movement_state)
  {
  case eST_DEACTIVATED:
      message["state"] = "deactivated";
    break;
  case eST_RESETTING:
      message["state"] = "resetting";
    break;
  case eST_RESETTED:
    message["state"] = "resetted, waiting for activation";
    break;
  case eST_FAULT: message["state"] = "fault";
    break;
  case eST_ENABLED:
      message["state"] = "enabled";
    break;
  case eST_PARTIALLY_ENABLED:
    message["state"] = "partially enabled";
     break;
  default:
    message["state"] = "unknown";
  };

  for (size_t i = 0; i < m_num_axes; ++i)
  {
    std::string joint = m_joint_descriptions[i];
    message["position"][joint] = m_joint_positions[i];
    message["speed"][joint] = m_joint_velocities[i];
    message["leds"][joint] = m_joint_errors[i];
    message["enabled"][joint] = m_joints_enabled[i];
    message["homed"][joint] = m_joints_homed[i];
  }

  return json_writer.write(message);
}


void SVHState::setMovementState(const int &movement_state)
{
  m_movement_state = static_cast<MovementState>(movement_state);
}


}} // NS end
