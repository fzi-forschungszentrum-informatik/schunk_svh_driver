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
 * \date    2014-5-21
 *
 */
//----------------------------------------------------------------------

#include "RobotState.h"


#include <cmath>


// In Case you prefer a version without JSONCPP dependency
//#include <iostream>
//#include <unistd.h>
//#include <sstream>
//#include <iomanip>



namespace icl_comm {
namespace websocket {


// Version without JSONCPP dependency.. but that seams to be reasonable enough
//  /*!
//   * \brief getStateJSON Returns the current robot state as JSON encoded String
//   * \return current robot state as JSON encoded String
//   */
//  std::string getStateJSON()
//  {
//    std::stringstream ss;
//    ss << std::setiosflags(std::ios::fixed) << std::setprecision(5);
//    ss <<   "{";
//    ss <<     "\"type\": \"roboter\",";
//    ss <<     "\"state\": \"";
//    switch (m_movement_state)
//    {
//    case eST_MOVING:
//      ss << "moving";
//      break;
//    case eST_WAITING:
//      ss << "waiting";
//      break;
//    default:
//      ss << "unknown";
//    };
//    ss << "\",";
//    ss <<     "\"position\": {";
//    for (int i = 0; i < m_num_axes; ++i)
//    {
//      ss <<       "\"joint"<<i<<"\": " << m_joint_positions[i] << (i < m_num_axes-1 ? ",":"");
//    }
//    ss <<     "},";
//    ss <<     "\"speed\": {";
//    for (int i = 0; i < m_num_axes; ++i)
//    {
//      ss <<       "\"joint"<<i<<"\": " << (m_joint_velocities[i]*m_tps) << (i < m_num_axes-1 ? ",":"");
//    }
//    ss <<     "},";
//    ss <<     "\"leds\": {";
//    for (int i = 0; i < m_num_axes; ++i)
//    {
//      ss <<       "\"joint"<<i<<"\": " << m_joint_errors[i] << (i < m_num_axes-1 ? ",":"");
//    }
//    ss <<     "}";
//    ss <<   "}";
//    return ss.str();
//  }

void RobotState::shuffleErrors()
{
  for (size_t i = 0; i < m_num_axes; ++i)
  {
    m_joint_errors[i] = static_cast<ErrorState>(rand()% eERR_DIMENSION);
    m_joints_homed[i] = static_cast<ErrorState>(rand()% eERR_DIMENSION);
    m_joints_enabled[i] = static_cast<ErrorState>(rand()% eERR_DIMENSION);
  }
}

void RobotState::simulateTick()
{
  // simple integration of positions
  for (size_t i = 0; i < m_num_axes; ++i)
  {
    m_joint_positions[i] += m_joint_velocities[i];
  }

  // Do Joint 0 differnt just to get a bit of variation
  m_joint_velocities[0] = std::sin(static_cast<double>(m_ticks)/m_tps)/m_tps;

  for (size_t i = 1; i < m_num_axes; ++i)
  {
    m_joint_velocities[i] = std::sin(static_cast<double>(-m_ticks)/m_tps)/m_tps;
  }

  ++m_ticks;

  // Shuffle Errors once per seconds (if the simulation is called accordingly)
  if (m_ticks % m_tps == 0)
  {
    shuffleErrors();
  }
}

















void RobotState::setJointPositions(const std::vector<double> &joint_positions)
{
  for (size_t i = 0; i < m_num_axes && i < joint_positions.size(); ++i)
  {
    m_joint_positions[i] = joint_positions[i]*m_input_to_rad_factor;
  }
}

void RobotState::setJointPosition(const double &joint_position,size_t joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
    m_joint_positions[joint] = joint_position*m_input_to_rad_factor;
  }
}


void RobotState::setJointPositions(const std::vector<int> &joint_positions)
{
  for (size_t i = 0; i < m_num_axes && i < joint_positions.size(); ++i)
  {
    m_joint_positions[i] = static_cast<double>(joint_positions[i]*m_input_to_rad_factor);
  }
}

void RobotState::setJointPosition(const int &joint_position,size_t joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
    m_joint_positions[joint] = joint_position*m_input_to_rad_factor;
  }
}

void RobotState::setJointVelocites(const std::vector<double> &joint_velocities)
{
  for (size_t i = 0; i < m_num_axes && i < joint_velocities.size(); ++i)
  {
    m_joint_velocities[i] = joint_velocities[i]*m_input_to_rad_factor;
  }
}

void RobotState::setJointVelocity(const double &joint_velocity,const size_t &joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
   m_joint_velocities[joint] = joint_velocity*m_input_to_rad_factor;
  }
}


void RobotState::setJointErrors(const std::vector<ErrorState> &joint_errors)
{
  for (size_t i = 0; i < m_num_axes && i < joint_errors.size(); ++i)
  {
    m_joint_errors[i] = joint_errors[i]?eERR_FAULT:eERR_NO_ERROR;
  }
}

void RobotState::setJointError(const ErrorState &joint_error,const size_t &joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
    m_joint_errors[joint] = joint_error?eERR_FAULT:eERR_NO_ERROR;
  }

}

void RobotState::setJointErrors(const std::vector<bool> &faults)
{
  std::vector<ErrorState> joint_errors(m_num_axes,eERR_NO_ERROR);
  for (size_t i = 0; i < m_num_axes && i < faults.size(); ++i)
  {
    joint_errors[i] = faults[i]?eERR_FAULT:eERR_NO_ERROR;
  }
  setJointErrors(joint_errors);
}

void RobotState::setJointError(const bool &fault, const size_t &joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
    m_joint_errors[joint] = fault?eERR_FAULT:eERR_NO_ERROR;
  }
}

void RobotState::setJointsEnabled(const std::vector<bool> &joints_enabled)
{
  for (size_t i = 0; i < m_num_axes && i < joints_enabled.size(); ++i)
  {
    m_joints_enabled[i] = joints_enabled[i]?eERR_NO_ERROR:eERR_FAULT;
  }

}

void RobotState::setJointEnabled(const bool &enabled,const size_t &joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
    m_joints_enabled[joint] = enabled?eERR_NO_ERROR:eERR_FAULT;
  }
}

void RobotState::setJointsHomed(const std::vector<bool> &joints_homed)
{
  for (size_t i = 0; i < m_num_axes && i < joints_homed.size(); ++i)
  {
    m_joints_homed[i] = joints_homed[i]?eERR_NO_ERROR:eERR_FAULT;
  }
}

void RobotState::setJointHomed(const bool &homed,const size_t &joint)
{
  if ((joint >= 0) && (joint < m_num_axes))
  {
    m_joints_homed[joint] = homed?eERR_NO_ERROR:eERR_FAULT;
  }
}



//void RobotState::setState(const std::vector<double> &joint_positions,const std::vector<double> &joint_velocities,const std::vector<bool> &faults,const MovementState &movement_state)
//{
//  setJointPositions(joint_positions);
//  setJointVelocites(joint_velocities);
//  setJointErrors(faults);
//  setMovementState(movement_state);
//}


}}
