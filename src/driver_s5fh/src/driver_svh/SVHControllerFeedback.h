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
 * This file contains the ControllerFeedback data structure that is used to
 * receive feedback of the hardware indicating the current position and
 * current values for each channel (finger)
 */
//----------------------------------------------------------------------
#ifndef SVHCONTROLLERFEEDBACK_H
#define SVHCONTROLLERFEEDBACK_H

#include <icl_comm/ByteOrderConversion.h>

namespace driver_svh {

/*!
 * \brief The SVHControllerFeedback saves the feedback of a single motor
 */
struct SVHControllerFeedback
{
  //! Returned position value of the motor [Ticks]
  int32_t position;
  //! Returned current value of the motor [mA]
  int16_t current;

  /*!
   * \brief standard constructor
   * \param _position Intitial position value, defaults to 0
   * \param _current Initital current value, defaults to 0
  **/
  SVHControllerFeedback(const int32_t& _position = 0,const int16_t& _current = 0):
    position(_position),
    current(_current)
  {}

  //! Compares two SVHControllerFeedback objects.
  bool operator == (const SVHControllerFeedback& other) const
  {
    return (position == other.position && current == other.current);
  }
};

/*!
 * \brief The SVHControllerFeedbackAllChannes saves the feedback of a all motors
 */
struct SVHControllerFeedbackAllChannels
{
  //! Vector holding multiple channels
  std::vector<SVHControllerFeedback> feedbacks;

  /*!
   * \brief Constructs a SVHControllerFeedbackAllChannels data structure from explicit ffedback elements
   * \param _feedback0 Feedback for the Thumb_Flexion
   * \param _feedback1 Feedback for the Thumb_Opposition
   * \param _feedback2 Feedback for the Index_Finger_Distal
   * \param _feedback3 Feedback for the Index_Finger_Proximal
   * \param _feedback4 Feedback for the Middle_Finger_Distal
   * \param _feedback5 Feedback for the Middle_Finger_Proximal
   * \param _feedback6 Feedback for the Ring_Finger
   * \param _feedback7 Feedback for the Pinky
   * \param _feedback8 Feedback for the Finger_Spread
   */
  SVHControllerFeedbackAllChannels(const SVHControllerFeedback& _feedback0,const SVHControllerFeedback& _feedback1,
                                    const SVHControllerFeedback& _feedback2,const SVHControllerFeedback& _feedback3,
                                    const SVHControllerFeedback& _feedback4,const SVHControllerFeedback& _feedback5,
                                    const SVHControllerFeedback& _feedback6,const SVHControllerFeedback& _feedback7,
                                    const SVHControllerFeedback& _feedback8)
  {
    feedbacks.push_back(_feedback0);
    feedbacks.push_back(_feedback1);
    feedbacks.push_back(_feedback2);
    feedbacks.push_back(_feedback3);
    feedbacks.push_back(_feedback4);
    feedbacks.push_back(_feedback5);
    feedbacks.push_back(_feedback6);
    feedbacks.push_back(_feedback7);
    feedbacks.push_back(_feedback8);
  }

  /*!
   * \brief Creates a SVHControllerFeedbackAllChannels structure from a vector
   * \param _feedbacks Vector filled with SVHControllerFeedback elements.
   * \note Alhough it is possible to create a feedback vector with more than 9 elements, that would be rather pointles as we only have 9 chanels less than 9 channels will result in only partial feedback
   */
  SVHControllerFeedbackAllChannels(std::vector<SVHControllerFeedback> _feedbacks)
  {
    feedbacks.insert(feedbacks.begin(),_feedbacks.begin(),_feedbacks.end());
  }

  /*!
   * \brief Constructs an empty SVHControllerFeedbackAllChannels objects, prefilled with 9 default channel feedbacks, mainly usefull for deserialization
   */
  SVHControllerFeedbackAllChannels():
    feedbacks(9)
  { }


  //! Compares two SVHControllerFeedbackAllChannels objects.
  bool operator == (const SVHControllerFeedbackAllChannels& other) const
  {
    return (feedbacks == other.feedbacks);
  }
};

//! Overload stream operator to easily serialize feedback data
inline icl_comm::ArrayBuilder& operator << (icl_comm::ArrayBuilder& ab,const SVHControllerFeedback& data)
{
  ab << data.position
     << data.current;
  return ab;
}


//! Overload stream operator to easily deserialize feedback data
inline icl_comm::ArrayBuilder& operator >> (icl_comm::ArrayBuilder& ab, SVHControllerFeedback& data)
{
  ab >> data.position
     >> data.current;
  return ab;
}

//! Output stream operator for easy output of feedback data
inline std::ostream& operator << (std::ostream& o, const SVHControllerFeedback& cf)
{
  o << "Pos: " << cf.position << " Cur: " << cf.current << std::endl;
  return o;
}


//! Overload stream operator to easily serialize all channel feedback data
inline icl_comm::ArrayBuilder& operator << (icl_comm::ArrayBuilder& ab,SVHControllerFeedbackAllChannels& data)
{
  // The Data is transmitted not channel by channel but rather position first, Currents afterwards for all channels
  for (std::vector<SVHControllerFeedback>::iterator it = data.feedbacks.begin() ; it != data.feedbacks.end(); ++it)
  {
    ab << it->position;
  }

  for (std::vector<SVHControllerFeedback>::iterator it = data.feedbacks.begin() ; it != data.feedbacks.end(); ++it)
  {
    ab << it->current;
  }
  return ab;
}


//! Overload stream operator to easily deserialize all channel feedback data
inline icl_comm::ArrayBuilder& operator >> (icl_comm::ArrayBuilder& ab, SVHControllerFeedbackAllChannels& data)
{
  // The Data is transmitted not channel by channel but rather position first, Currents afterwards for all channels
  for (std::vector<SVHControllerFeedback>::iterator it = data.feedbacks.begin() ; it != data.feedbacks.end(); ++it)
  {
    ab >> it->position;
  }

  for (std::vector<SVHControllerFeedback>::iterator it = data.feedbacks.begin() ; it != data.feedbacks.end(); ++it)
  {
    ab >> it->current;
  }
  return ab;
}

//! Output stream operator for easy output of all channel feedback data
inline std::ostream& operator << (std::ostream& o, const SVHControllerFeedbackAllChannels& cf)
{
  o << "Feedbacks: " ;
  unsigned int i = 0;
  for (std::vector<SVHControllerFeedback>::const_iterator it = cf.feedbacks.begin() ; it != cf.feedbacks.end(); ++it,++i)
  {
    o << "Chan " << i << " : "<< *it;
  }
  o << std::endl;
  return o;
}

}

#endif // SVHCONTROLLERFEEDBACK_H
