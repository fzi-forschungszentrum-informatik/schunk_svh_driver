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

#ifndef SVHSTATE_H
#define SVHSTATE_H

#include <icl_comm_websocket/RobotState.h>

namespace icl_comm {
namespace websocket {

class SVHState : public RobotState
{
public:

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


  SVHState();

  /*!
   * \brief getStateJSON Returns the current robot state as JSON encoded String
   * \return current robot state as JSON encoded String
   */
  virtual std::string getStateJSON();

  virtual void setMovementState(const int &movement_state);

private:
  //! Current movement state of the whole robot
  MovementState m_movement_state;

  //! Description values to get the corresponding enum value to a channel
  static const std::string m_joint_descriptions[];
};


}} // NS End

#endif // SVHSTATE_H
