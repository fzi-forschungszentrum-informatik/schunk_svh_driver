// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
*
* \author  Florian Kuhnt <kuhnt@fzi.de>
* \date    12.08.2013
*
*/
//----------------------------------------------------------------------

#ifndef ICL_CORE_SOURCESINK_IMPORT_EXPORT_H_INCLUDED
#define ICL_CORE_SOURCESINK_IMPORT_EXPORT_H_INCLUDED

#if defined(_SYSTEM_WIN32_) && !defined(_IC_STATIC_)
#  pragma warning ( disable : 4251 )

# if defined ICL_CORE_SOURCESINK_EXPORT_SYMBOLS
#  define ICL_CORE_SOURCESINK_IMPORT_EXPORT __declspec(dllexport)
# else
#  define ICL_CORE_SOURCESINK_IMPORT_EXPORT __declspec(dllimport)
# endif

#elif defined(__GNUC__) && (__GNUC__ > 3) && !defined(_IC_STATIC_)

# define ICL_CORE_SOURCESINK_IMPORT_EXPORT __attribute__ ((visibility("default")))

#else

# define ICL_CORE_SOURCESINK_IMPORT_EXPORT

#endif

#endif // _icl_core_plugin_ImportExport_h_
