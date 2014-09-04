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

#ifndef ICL_CORE_SOURCESINK_POSITIONRESOLVERBINARY_H
#define ICL_CORE_SOURCESINK_POSITIONRESOLVERBINARY_H

#include "PositionResolver.h"

namespace icl_core {
namespace sourcesink {


/**
 * @brief The PositionResolverBinary class
 *
 * This is a position resolver that additionally holds the binary position value for each frame.
 */
class ICL_CORE_SOURCESINK_IMPORT_EXPORT PositionResolverBinary : public PositionResolver
{
public:
  PositionResolverBinary();

  typedef uint64_t BinaryPosition;

  void addFrame(boost::posix_time::ptime time, int frame_number, BinaryPosition binary_position);

  void clear();

  BinaryPosition getBinaryPosition(size_t index);

protected:
  std::vector<BinaryPosition>& binaryPositions();
  const std::vector<BinaryPosition>& binaryPositions() const;

private:
  virtual void readFromFile(std::ifstream &stream);
  virtual void writeToFile(std::ofstream &stream) const;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int)
  {
    ar & timestamps();
    ar & frameNumbers();
    ar & binaryPositions();
  }

  std::vector<BinaryPosition> m_binary_positions;
};


}
}

#endif // ICL_CORE_SOURCESINK_POSITIONRESOLVERBINARY_H
