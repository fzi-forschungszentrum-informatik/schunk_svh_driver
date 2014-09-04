// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \date    2014-01-30
 *
 * Defines used for windows DLL support.
 */
//----------------------------------------------------------------------
#ifndef DRIVER_SVH_IMPORT_EXPORT_H_INCLUDED
#define DRIVER_SVH_IMPORT_EXPORT_H_INCLUDED

#if defined(_SYSTEM_WIN32_) && !defined(_IC_STATIC_)
#  pragma warning ( disable : 4251 )

# if defined DRIVER_SVH_EXPORT_SYMBOLS
#  define DRIVER_SVH_IMPORT_EXPORT __declspec(dllexport)
# else
#  define DRIVER_SVH_IMPORT_EXPORT __declspec(dllimport)
# endif

#elif defined(__GNUC__) && (__GNUC__ > 3) && !defined(_IC_STATIC_)

# define DRIVER_SVH_IMPORT_EXPORT __attribute__ ((visibility("default")))

#else

# define DRIVER_SVH_IMPORT_EXPORT

#endif

#endif
