// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Georg Heppner
 * \date    2014-02-03
 * \date    2014-07-16
 *
 * This file contains the SVHFirmwareInfo data structure that is used to
 * receive information about the current firmware settings
 */
//----------------------------------------------------------------------
#ifndef SVHFIRMWAREINFO_H
#define SVHFIRMWAREINFO_H

namespace driver_svh {

/*!
 * \brief The SVHFirmwareInfo  holds the data of a firmware response from the hardware
 */
struct SVHFirmwareInfo
{
  //! 4 bytes identifier
  std::string svh;
  //! Major version number
  uint16_t version_major;
  //! Minor version number
  uint16_t version_minor;
  //! 48 bytes! of text (free)
  std::string text;

  //! Compares two SVHFirmware objects.
  bool operator == (const SVHFirmwareInfo& other) const
  {
    return (version_major == other.version_major && version_minor == other.version_minor);
  }
};

//! overload stream operator to easily serialize firmware data
inline icl_comm::ArrayBuilder& operator << (icl_comm::ArrayBuilder& ab, SVHFirmwareInfo& data)
{
  // Stream operator can not handle arrays (due to missing size information) to make things easy we just copy the data around. Feel free to do something else
  // Todo : Can the operator handle this without conversion? Also .. string uses char.. and not unsigned char.. but that is also 8 bit so.. we are save.. maybe we have to use std::basic_string<unsinge_char> ... try it..
  std::vector<uint8_t> text(48);
  std::vector<uint8_t> svh(4);
  svh.insert(svh.begin(),data.svh.begin(),data.svh.end());
  text.insert(text.begin(),data.text.begin(),data.text.end());

  ab << svh
     << data.version_major
     << data.version_minor
     << text;
  return ab;
}



//! overload stream operator to easily serialize firmware data
inline icl_comm::ArrayBuilder& operator >> (icl_comm::ArrayBuilder& ab, SVHFirmwareInfo& data)
{
  // Stream operator can not handle arrays (due to missing size information) to make things easy we just copy the data around. Feel free to do something else
  std::vector<uint8_t> text(48);
  std::vector<uint8_t> svh(5);

  ab >> svh
     >> data.version_major
     >> data.version_minor
     >> text;


  data.text = std::string(text.begin(),text.end());
  data.svh = std::string(svh.begin(),svh.end());
//  std::copy (text.begin(),text.end(),data.text);
//  std::copy (svh.begin(),svh.end(),data.svh);

  return ab;
}

//! Output Stream operator for easy output of the firmware information
inline std::ostream& operator << (std::ostream& o, const SVHFirmwareInfo& fw)
{
  o << fw.svh.c_str()  << " " << fw.version_major << "." << fw.version_minor << " : " << fw.text.c_str() << endl;
  return o;
}



}



#endif // SVHFIRMWAREINFO_H
