// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    11.12.2013
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_POSITIONRESOLVEREXTRA_H
#define ICL_CORE_SOURCESINK_POSITIONRESOLVEREXTRA_H

#include "PositionResolver.h"

namespace icl_core {
namespace sourcesink {


/**
 * @brief The PositionResolverExtra class
 *
 * This is a position resolver that additionally holds a data field for each frame.
 */
template <class T>
class PositionResolverExtra : public PositionResolver
{
public:
  PositionResolverExtra();

  typedef T value_type;

  void addFrame(boost::posix_time::ptime time, int frame_number, value_type extra_data = value_type());

  void clear();

  value_type getExtraData(size_t index);

protected:
  std::vector<value_type>& extraData();
  const std::vector<value_type>& extraData() const;

private:
  virtual void readFromFile(std::ifstream &stream);
  virtual void writeToFile(std::ofstream &stream) const;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int)
  {
    ar & timestamps();
    ar & frameNumbers();
    ar & extraData();
  }

  std::vector<value_type> m_extra_data;
};


}
}

#include "PositionResolverExtra.hpp"

#endif // ICL_CORE_SOURCESINK_POSITIONRESOLVEREXTRA_H
