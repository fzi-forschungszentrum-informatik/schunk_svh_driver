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
 * \date    2014-5-20
 *
 */
//----------------------------------------------------------------------

#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <vector>
#include <stdlib.h>
#include <string>




namespace icl_comm {
namespace websocket {


/*!
 * \brief The RobotState class is a helper class to hold the relevant information about the current robot state
 * A robo has many axes, a movement state, fault states for every joint. In Short this class was meant to be used for the
 * SCHUNK LWA4-P Together with canopen. However it might also be used with other arms
 */
class RobotState
{
public:




  /*!
   * \brief The ErrorState enum describes the possible error states a joint may indicate
   */
  enum ErrorState
  {
    eERR_NO_ERROR = 0,
    eERR_WARNING = 1,
    eERR_FAULT = 2,
    eERR_DIMENSION
  };

  /*!
   * \brief RobotState Object to hold the current state of a robotic arm (i.e LWA4P)
   * \param num_axes Number of axes to send out
   * \param ticks_per_second Number of simulation stepds per second if the simulation should be used. Default = 100. Not needed if you use real data
   */
  RobotState(size_t num_axes) :
    m_num_axes(num_axes),
    m_joint_positions(num_axes,0.0),
    m_joint_velocities(num_axes,0.0),
    m_joint_errors(num_axes,eERR_NO_ERROR),
    m_joints_homed(num_axes,eERR_WARNING),
    m_joints_enabled(num_axes,eERR_WARNING),
    m_tps(100),
    m_ticks(0),
    m_input_to_rad_factor(1)
  { }

  /*!
   * \brief simulateTick Simulates one step of a dummy simulation to generate random output values
   */
  virtual void simulateTick();

  //! Shuffles the Error values to contain random values from the pool of possible errors (used for simulation)
  virtual void shuffleErrors();

  /*!
   * \brief getStateJSON Returns the current robot state as JSON encoded String
   * \return current robot state as JSON encoded String
   */
  virtual std::string getStateJSON() = 0;




  /*!
   * \brief setJointPositions Sets the current joint positionts. Only num_axes elements will be used. Unset element will retain their previous value
   * \param joint_positions new joint positions in your format, this will be multiplied by the toRadMultiplier
   */
  void setJointPositions(const std::vector<double> &joint_positions);

  /*!
   * \brief setJointPosition sets the current koint position of a particular joint.
   * \param joint_position Current Position in your format, this will be multiplied by the toRadMultiplier
   * \param joint index of the joint (starts with zero, obviously)
   */
  void setJointPosition(const double &joint_position,size_t joint);

  /*!
   * \brief setJointPositions Sets the current joint positionts. Only num_axes elements will be used. Unset element will retain their previous value
   * \param joint_positions new joint positions in your format, this will be multiplied by the toRadMultiplier
   */
  void setJointPositions(const std::vector<int> &joint_positions);

  /*!
   * \brief setJointPosition sets the current koint position of a particular joint.
   * \param joint_position Current Position in your format, this will be multiplied by the toRadMultiplier
   * \param joint index of the joint (starts with zero, obviously)
   */
  void setJointPosition(const int &joint_position,size_t joint);

  /*!
   * \brief setJointVelocties Sets the current joint velocities. Only num_axes elements will be used. Unset element will retain their previous value
   * \param joint_velocities new joint velocities in your format, this will be multiplied by the toRadMultiplier
   */
  void setJointVelocites(const std::vector<double> &joint_velocities);

  /*!
   * \brief setJointVelocty sets the current joint velocity of a single joint
   * \param joint_velocit< new joint velocities inyour format, this will be multiplied by the toRadMultiplier
   */
  void setJointVelocity(const double &joint_velocity,const size_t &joint);

  /*!
   * \brief setJointErrors Sets the current joint error state. Only num_axes elements will be used. Unset element will retain their previous value
   * \param joint_errors new joint error state
   */
  void setJointErrors(const std::vector<ErrorState> &joint_errors);

  /*!
   * \brief setJointError sets the current joint error state of a particular joint
   * \param joint_error new error state to set
   * \param joint the joint to set the error for
   */
  void setJointError(const ErrorState &joint_error,const size_t &joint);

  /*!
   * \brief setJointErrors Sets the current joint errors in a simplified way (only fault or no fault) if no further information is available
   * \param faults true if joint had a fault, false otherwise
   */
  void setJointErrors(const std::vector<bool> &faults);

  /*!
   * \brief setJointError Simplified version of error setting for a particular joint if only fault or no fault is supported
   * \param fault true if joint had a fault, false otherwise
   * \param joint the joint to set the error for
   */
  void setJointError(const bool &fault, const size_t &joint);


  void setJointsEnabled(const std::vector<bool> &joints_enabled);

  void setJointEnabled(const bool &enabled,const size_t &joint);

  void setJointsHomed(const std::vector<bool> &joints_homed);

  void setJointHomed(const bool &homed,const size_t &joint);




  virtual void setMovementState(const int &movement_state) = 0;


  //virtual void setState(const std::vector<double> &joint_positions,const std::vector<double> &joint_velocities,const std::vector<bool> &faults,const MovementState &movement_state);


  void setTps(unsigned int tps)
  {
    m_tps = tps;
  }

  size_t getNumAxes() const
  {
    return m_num_axes;
  }

  double getInputToRadFactor() const
  {
    return m_input_to_rad_factor;
  }

  void setInputToRadFactor(double input_to_rad_factor)
  {
    m_input_to_rad_factor = input_to_rad_factor;
  }

protected:
  //! How many axes is this robot defined for
  size_t m_num_axes;

  //! Position of the joints(rad)
  std::vector<double> m_joint_positions;

  //! Speed of the joints (rad/sec)
  std::vector<double> m_joint_velocities;

  //! Error indication for individual joint
  std::vector<ErrorState> m_joint_errors;

  //! Indication if a joint is homed
  std::vector<ErrorState> m_joints_homed;

  //! Indication if a joint is enabled
  std::vector<ErrorState> m_joints_enabled;

  //! Ticks per Second can be used to simulate data in the robot
  unsigned int  m_tps;
  //! Current tick counter for simulation purposes
  unsigned int  m_ticks;

  //! in case the input positions are not Rad we use this factor
  double m_input_to_rad_factor;

};


}}

#endif // ROBOTSTATE_H
