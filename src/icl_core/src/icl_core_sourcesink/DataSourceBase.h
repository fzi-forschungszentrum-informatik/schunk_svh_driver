// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2013-10-23
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_DATASOURCEBASE_H
#define ICL_CORE_SOURCESINK_DATASOURCEBASE_H

#include "DataSourceSink.h"
#include "boost/date_time.hpp"
#include "Position.h"

namespace icl_core {
namespace sourcesink {

class ICL_CORE_SOURCESINK_IMPORT_EXPORT DataSourceBase : public DataSourceSink
{
public:
  typedef boost::shared_ptr<DataSourceBase> Ptr;



  DataSourceBase ( const std::string &identifier = "DataSource" );

  virtual ~DataSourceBase();

  /** Filename of the video */
  virtual std::string fileName() const = 0;

  /**
   * @brief size
   * @return Total number of frames
   * @note This function must be overloded by a seekable source
   */
  virtual int size() const = 0;

  /**
   * @brief isSeekable
   * @return true iff source is seekable
   */
  virtual bool isSeekable() const = 0;


  /// copied from SynchronizedSource - use Position instead of ptime?

  /** Should return the current time - now() for cameras and position for data files */
  /**
   * @brief getCurrentTimestamp
   * @return Timestamp of current frame
   */
  virtual boost::posix_time::ptime getCurrentTimestamp() const;

  /** Should return the next frames' timestamp - now() + frame delay for cameras */
  /**
   * @brief getNextTime
   * @return Timestamp of the next frame (index+1)
   */
  virtual boost::posix_time::ptime getNextTime() const;

  /** Should return the first frames' timestamp - now() for cameras */
  /**
   * @brief getFirstTime
   * @return The first frame's timestamp
   */
  virtual boost::posix_time::ptime getFirstTime() const;

  /** Should return the last frames' timestamp - now() for cameras */
  /**
   * @brief getLastTime
   * @return The last frame's timestamp
   */
  virtual boost::posix_time::ptime getLastTime() const;

  /**
   * Checks given timestamp
   * @param timestamp Timestamp to validate
   * @return True iff timestamp is lower equal than last
   * timestamp and greater equal than first timestamp
   */
  virtual bool isTimestampValid ( const boost::posix_time::ptime &timestamp ) const;

  /**
   * @brief seekNext
   * seek to the next frame and read it into a buffer.
   *
   * \todo replace seekNext by a function mapping on processOnce()
   *
   * A SeekableDataSource implements this returning seek(getCurrentIndex()+1)
   *
   * @return true iff the seek was successful.
   */
  virtual bool seekNext() = 0;

  /**
   * Jump to the given position
   * @param position Jump target, evaluated as follows:
   * If position.index is a valid index, it jumps to the frame with that index
   * Otherwise, if position.frame_number is a valid frame_number, it jumps to
   * the frame with that frame number
   * Otherwise, if position.timestamp is a non-special timestamp within the range
   * of the video, it jumps to the frame with that timestamp
   * @note This function must be overloded by a seekable source
   */
  virtual bool seek(const Position &position) = 0;

  /**
   *  Convenience method for relative seeking, equivalent to calling Seek(getCurrentIndex() + offset)
   *  Can be overloaded for e.g. performance improvements.
   */
  virtual void seekRelative(int index_offset);

  /**
   * Resolves position information for a frame
   * @param position If position.index is a valid index, it sets the frame_number
   * and timestamp of the frame with that index
   * Otherwise, if position.frame_number is a valid frame_number, it sets the index
   * and timestamp of the frame with that frame number
   * Otherwise, if position.timestamp is a non-special timestamp within the range
   * of the video, it sets the index and the frame_number of the frame with that timestamp.
   * If you can't implement this function just return false.
   * @return True indicates a suitable frame was found and at least one information set
   * @note This function must be overloded by a seekable source
   */
  virtual bool resolve(Position &position) const = 0;

  virtual boost::posix_time::ptime getTimestamp (const int &frameNumber) const;
  virtual int getFramenumber(const boost::posix_time::ptime &timestamp) const;

  /**
   * @brief getCurrentIndex
   * @return Index of the current frame
   * @note This function must be overloded by a seekable source
   */
  virtual int getCurrentIndex() const = 0;

};

}
}

#endif // ICL_CORE_SOURCESINK_DATASOURCEBASE_H
