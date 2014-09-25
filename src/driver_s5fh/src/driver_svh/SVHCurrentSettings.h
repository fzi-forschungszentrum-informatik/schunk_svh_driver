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
  //! Proportional gain
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

  //!
  //! \brief SVHCurrentSettings Construct current settings from a plain vector if the Vector is smaler than 10 values will be filled with 0.0
  //! \param cur_settings Vector of floats in the order: wmn, wmx, ky, dt, imn, imx, kp, ki , umn, umx
  //!
  SVHCurrentSettings(const std::vector<float>& cur_settings)
  {
    size_t size = cur_settings.size();

    // is there a nicer way to do this? Please tell me if there is :)
    wmn = (size > 0) ? cur_settings[0] : 0.0;
    wmx = (size > 1) ? cur_settings[1] : 0.0;
    ky  = (size > 2) ? cur_settings[2] : 0.0;
    dt  = (size > 3) ? cur_settings[3] : 0.0;
    imn = (size > 4) ? cur_settings[4] : 0.0;
    imx = (size > 5) ? cur_settings[5] : 0.0;
    kp  = (size > 6) ? cur_settings[6] : 0.0;
    ki  = (size > 7) ? cur_settings[7] : 0.0;
    umn = (size > 8) ? cur_settings[8] : 0.0;
    umx = (size > 9) ? cur_settings[9] : 0.0;
  }

  //!
  //! \brief SVHCurrentSettings Constructs a new current settings object to configure the current controller of a finger
  //! \param wmn Reference signal minimum value
  //! \param wmx Reference signal maximum value
  //! \param ky measurement scaling
  //! \param dt time base of controller
  //! \param imn Integral windup minimum value
  //! \param imx Integral windup maximum value
  //! \param kp  Proportional gain
  //! \param ki  Integral gain
  //! \param umn Output limiter min
  //! \param umx Output limiter max
  //!
  SVHCurrentSettings(const float& wmn,const float& wmx,const float& ky,const float& dt,const float& imn,
                     const float& imx,const float& kp,const float& ki,const float& umn,const float& umx):
    wmn(wmn),wmx(wmx),ky (ky),dt (dt),imn(imn), imx(imx), kp (kp), ki (ki), umn(umn), umx(umx)
  {}

  //!
  //! \brief SVHCurrentSettings Default constructor, initalizes everything to zero
  //!
  SVHCurrentSettings():
    wmn(0.0),wmx(0.0),ky (0.0), dt (0.0), imn(0.0), imx(0.0), kp (0.0), ki (0.0), umn(0.0), umx(0.0)
  {}


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
