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
#include <driver_svh/Logging.h>
#include <driver_svh/SVHFingerManager.h>

namespace driver_svh {

SVHFeedbackPollingThread::SVHFeedbackPollingThread(const TimeSpan& period, SVHFingerManager* finger_manager)
  : PeriodicThread("SVHReceiveThread", period),
    m_finger_manager(finger_manager)
{
}

void SVHFeedbackPollingThread::run()
{
  while (execute())
  {
    if (m_finger_manager != NULL)
    {
      if (m_finger_manager->isConnected())
      {
        m_finger_manager->requestControllerFeedback(eSVH_ALL);

        // This would inform a websocket server about new states, disregard that.
        #ifdef _IC_BUILDER_ICL_COMM_WEBSOCKET_
        m_finger_manager->updateWebSocket();
        #endif // _IC_BUILDER_ICL_COMM_WEBSOCKET_
      }
      else
      {
        LOGGING_WARNING_C(DriverSVH, SVHFeedbackPollingThread, "SCHUNK five finger hand is not connected!" << endl);
      }
    }
    else
    {
      LOGGING_WARNING_C(DriverSVH, SVHFeedbackPollingThread, "Pointer to FingerManager is NULL!" << endl);
    }

    // Wait for the thread period so that the timing is in sync.
    waitPeriod();
  }
}

}
