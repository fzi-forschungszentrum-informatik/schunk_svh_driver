// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Klaus Uhl <uhl@fzi.de>
 * \date    2010-08-05
 *
 * \brief   Contains misc hacks.
 *
 */
//----------------------------------------------------------------------

// This definition is needed for Insure++ to be able to link
// code which uses IOCTL calls.
// Remark: This definition is not guarded with __INSURE__ so that
// partially instrumented builds also work!
unsigned int __invalid_size_argument_for_IOC;
