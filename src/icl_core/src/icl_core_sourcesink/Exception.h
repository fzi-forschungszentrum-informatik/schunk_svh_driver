// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Ralf Kohlhaas <kohlhaas@fzi.de>
* \date    2013-10-15
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCE_SINK_EXCEPTION_H
#define ICL_CORE_SOURCE_SINK_EXCEPTION_H


#include <string>
#include <exception>


namespace icl_core {
namespace sourcesink {



/*!
 * Common exception type used by icl_core_source_sink
 */
class Exception : public std::exception
{
public:
  /** Constructor, setting type to GENERIC_ERROR */
  explicit Exception(const std::string &what_arg) throw() :
    m_what_arg ( what_arg )
  {
  }

  Exception(const Exception &other) throw() :
    m_what_arg(other.m_what_arg)
  {
  }

  Exception& operator=(const Exception &other) throw()
  {
    if (this != &other)
    {
      m_what_arg = other.m_what_arg;
    }
    return *this;
  }

  /** Destructor */
  virtual ~Exception() throw()
  {
  }


  /** @see std::exception#what */
  virtual const char* what() const throw()
  {
    return m_what_arg.c_str();
  }


  /** Get a description string for the exception */
  std::string getDescription() const
  {
    return m_what_arg;
  }


protected:
  /** Error description */
  std::string m_what_arg;
};

}
}

#endif // ICL_CORE_SOURCE_SINK_EXCEPTION_H

