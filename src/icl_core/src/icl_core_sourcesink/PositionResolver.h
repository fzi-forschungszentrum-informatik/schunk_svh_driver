// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2013-10-16
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_POSITIONRESOLVER_H
#define ICL_CORE_SOURCESINK_POSITIONRESOLVER_H

#include "SeekableDataSource.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

namespace icl_core {
namespace sourcesink {


class ICL_CORE_SOURCESINK_IMPORT_EXPORT PositionResolver
{
public:
  static const std::string TYPICAL_FILE_SUFFIX;

  PositionResolver();

  bool resolve(Position &position) const;

  void addFrame(boost::posix_time::ptime time, int frame_number);

  size_t size() const;

  void clear();

  /**
    * Restores the internal state from the given file. Returns true
    * if the file exists and can be loaded
   */
  bool load(const std::string &filename);

  /**
   * Stores the internal state to the given file, overriding any existing
   * file with the same name. Returns true if the state was written correctly,
   * false otherwise (e.g. parent directory does not exist, no write access)
   * @param filename File to write to. Overwrites any existing file with the same name
   */
  bool save(const std::string &filename) const;


protected:
  std::vector<boost::posix_time::ptime>& timestamps();
  const std::vector<boost::posix_time::ptime>& timestamps() const;

  std::vector<int>& frameNumbers();
  const std::vector<int>& frameNumbers() const;

private:
  virtual void readFromFile(std::ifstream &stream);
  virtual void writeToFile(std::ofstream &stream) const;

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int)
  {
    ar & m_timestamps;
    ar & m_frame_numbers;
  }

  std::vector<boost::posix_time::ptime> m_timestamps;
  std::vector<int> m_frame_numbers;
};


}
}

#endif // ICL_CORE_SOURCESINK_POSITIONRESOLVER_H
