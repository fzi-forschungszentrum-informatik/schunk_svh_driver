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

#ifndef WSBROADCASTER_H
#define WSBROADCASTER_H

#include <icl_comm_websocket/UnixSocket.h>

#include <icl_comm_websocket/RobotState.h>
#include <icl_comm_websocket/LWA4PState.h>
#include <icl_comm_websocket/SVHState.h>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/shared_ptr.hpp>

namespace icl_comm {
namespace websocket {

class WsBroadcaster
{
public:

  enum RobotType
  {
    eRT_LWA4P,
    eRT_SVH
  };

  WsBroadcaster(const RobotType &robot_type = eRT_LWA4P,const std::string &path="/tmp/ws_broadcaster")
    : socket(),
      m_path(path),
      send_error_counter(0),
      reset_error_counter(0)
  {
    switch (robot_type)
    {
      case eRT_SVH:
        robot = boost::shared_ptr<SVHState>(new SVHState());
        break;
      case eRT_LWA4P:
      default:
        robot = boost::shared_ptr<LWA4PState>(new LWA4PState());
        break;
    }

  }

  ~WsBroadcaster()
  {
    stopSimulation();
  }

  bool sendState();

  /*!
   * \brief simulateRobot Very simple thread to run the robot in simulation mode
   * \param cycle_time_ms cycle time for the thread. Every cycle_time_ms a step of the "simulation" is run
   */
  void simulateRobot(const int &cycle_time_ms)
  {
    while (true)
    {
      robot->simulateTick();
      boost::this_thread::sleep(boost::posix_time::milliseconds(cycle_time_ms));
      sendState();
    }
  }

  /*!
   * \brief startSimulation Starts a thread to run the robot simulation
   * \param cycle_time_ms cycle time for the thread. Every cycle_time_ms a step of the "simulation" is run
   */
  void startSimulation(const int &cycle_time_ms)
  {
      robot->setTps(1000/cycle_time_ms);
      robot->setJointPositions(std::vector<double>(robot->getNumAxes(),0.0));
      m_simulation_thread = boost::thread(boost::bind(&WsBroadcaster::simulateRobot,this,cycle_time_ms));
  }

  /*!
   * \brief stopSimulation Stops the "simulation" of the robot
   */
  void stopSimulation()
  {
    m_simulation_thread.interrupt();
    m_simulation_thread.join();
  }


 // As the robot state and possible later states are just meant as a data storage for everyone to write their values to they are public for convenience

 //! Robot state representing the current state of the robot in terms of diagnostics
 boost::shared_ptr<RobotState> robot;

private:
 //! Unix Socket to communicate with the Websocket server
  boost::shared_ptr<UnixSocket> socket;
  std::string m_path;
  size_t send_error_counter;
  size_t reset_error_counter;


 //! hread to simulate the robot output
 boost::thread m_simulation_thread;
};

}} //NS end

#endif // WSBROADCASTER_H
