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
 * \date    2014-02-17
 * \date    2014-07-16
 *
 * This file contains the FeedBackpollingthread.
 * The hardware itself will not send data all the time, only once in response
 * to each packet sent. As the desired behaviour is to get constant position feedback
 * of the fingers we trigger a controllerfeedback periodically to receive continious data.
 * The feedback polling thread is implemented in this file.
 */
//----------------------------------------------------------------------
#ifndef DRIVER_SVH_SVH_FEEDBACK_POLLING_THREAD_H_INCLUDED
#define DRIVER_SVH_SVH_FEEDBACK_POLLING_THREAD_H_INCLUDED

#include <icl_core/TimeSpan.h>
#include <icl_core_thread/PeriodicThread.h>
#include <driver_svh/SVHController.h>

using icl_core::TimeSpan;
using icl_core::thread::PeriodicThread;

namespace driver_svh {

// forward declaration as the fingermanager already uses this class
class SVHFingerManager;

/*!
 * \brief Thread for periodically requesting feedback messages from the SCHUNK five finger hand.
 */
class SVHFeedbackPollingThread : public PeriodicThread
{
public:
  /*!
   * \brief SVHFeedbackPollingThread constructs a new thread to poll the feedback of all fingers periodically
   * \param period timespan after which the thread should be woken up
   * \param finger_manager reference to the fingermanager which functions are used to do the polling
   */
  SVHFeedbackPollingThread(const TimeSpan& period, SVHFingerManager* finger_manager);

  //! default DTOR
  virtual ~SVHFeedbackPollingThread() {}

  //! run method of the thread
  virtual void run();

private:

  //! pointer to SCHUNK five finger hand fingermanager object
  SVHFingerManager* m_finger_manager;

};

}

#endif
