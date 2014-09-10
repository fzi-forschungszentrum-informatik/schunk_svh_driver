// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the SCHUNK SVH Driver suite.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE.txt in the top
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
 * This file contains the CurrentSettings data structure that is used to
 * send and receive the settings for the current controller in the position/current cascade
 */
//----------------------------------------------------------------------
#ifndef SVHCURRENTSETTINGS_H
#define SVHCURRENTSETTINGS_H

#include <icl_comm/ByteOrderConversion.h>

namespace driver_svh {

/*!
 * \brief The SVHCurrentSettings save the current controller paramters for a single motor
 */
struct SVHCurrentSettings
{
  //! Reference signal minimum value
  float wmn;
  //! Reference signal maximum value
  float wmx;
  //! measurement scaling
  float ky;
  //! time base of controller
  float dt;
  //! Integral windup minimum value
  float imn;
  //! Integral windup maximum value
  float imx;
  //! proportional gain
  float kp;
  //! Integral gain
  float ki;
  //! Output limiter min
  float umn;
  //! Output limiter max
  float umx;

  //! Compares two SVHCurrentSettings objects.
  bool operator == (const SVHCurrentSettings& other) const
  {
    return
      (wmn == other.wmn
       && wmx == other.wmx
       && ky == other.ky
       && dt == other.dt
       && imn == other.imn
       && imx == other.imx
       && kp == other.kp
       && ki == other.ki
       && umn == other.umn
       && umx == other.umx
       );
  }

};

//! overload stream operator to easily serialize current settings data
inline icl_comm::ArrayBuilder& operator << (icl_comm::ArrayBuilder& ab, const SVHCurrentSettings& data)
{
  ab << data.wmn
     << data.wmx
     << data.ky
     << data.dt
     << data.imn
     << data.imx
     << data.kp
     << data.ki
     << data.umn
     << data.umx;
  return ab;
}

//! overload stream operator to easily serialize current settings data
inline icl_comm::ArrayBuilder& operator >> (icl_comm::ArrayBuilder& ab, SVHCurrentSettings& data)
{
  ab >> data.wmn
     >> data.wmx
     >> data.ky
     >> data.dt
     >> data.imn
     >> data.imx
     >> data.kp
     >> data.ki
     >> data.umn
     >> data.umx;
  return ab;

  return ab;
}

//! Output stream operator for easy output of current settings
inline std::ostream& operator << (std::ostream& o, const SVHCurrentSettings& cs)
{
  o << "wmn "<< cs.wmn << " "
    << "wmx "<< cs.wmx << " "
    << "ky " << cs.ky  << " "
    << "dt " << cs.dt  << " "
    << "imn "<< cs.imn << " "
    << "imx "<< cs.imx << " "
    << "kp " << cs.kp  << " "
    << "ki " << cs.ki  << " "
    << "umn "<< cs.umn << " "
    << "umx "<< cs.umx << " "
    << std::endl;
  return o;
}


}
#endif // SVHCURRENTSETTINGS_H
