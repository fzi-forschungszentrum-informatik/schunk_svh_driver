// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of FZIs ic_workspace.
//
// This program is free software licensed under the LGPL
// (GNU LESSER GENERAL PUBLIC LICENSE Version 3).
// You can find a copy of this license in LICENSE folder in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lars Pfotzer
 * \author  Georg Heppner
 * \date    2014-02-03
 *
 */
//----------------------------------------------------------------------
#ifndef ICL_COMM_BYTE_ORDER_CONVERISON_H_INCLUDED
#define ICL_COMM_BYTE_ORDER_CONVERISON_H_INCLUDED

#include <icl_core/BaseTypes.h>
#include <icl_core/Vector.h>

#include "icl_comm/ImportExport.h"

#include <boost/detail/endian.hpp>
#include <iostream>
#include <iomanip>
#include <assert.h>

// For type limiting
#include <boost/type_traits/is_arithmetic.hpp>

namespace icl_comm {

//! template function for adding data to an array while converting everything into correct endianess
template <typename T>
size_t toLittleEndian(const T& data, std::vector<uint8_t>& array, size_t& write_pos)
{
  // Resize the target array in case it it to small to avoid out of bounds acces
  if (write_pos + sizeof(T) > array.size())
  {
    // TODO: Remove Debug
    //std::cout << "To Little Endian has to extend the Array. Current array Size: "<< (int)array.size() << " Pos: " << (int)pos << " Size_T: "<< sizeof(T) << " New array Size: "<< (int)(pos + sizeof(T)) << std::endl;
    array.resize(write_pos + sizeof(T) );
  }

  // Endianess Conversion
  for (size_t i= 0; i< sizeof(T);++i)
  {
    // Copy each byte into the bytearray, always convert byte order to little endian regardles of source architecture
    array[write_pos+i] = static_cast<uint8_t>((data>>(i*8)) & 0xFF);
  }

  return write_pos + sizeof(T);
}

//! Template specialization for float as these have to be handled seperately
template <>
ICL_COMM_IMPORT_EXPORT
size_t toLittleEndian<float>(const float& data, std::vector<uint8_t>& array, size_t& write_pos);

//! Template specialization for float as these have to be handled seperately
template <>
ICL_COMM_IMPORT_EXPORT
size_t toLittleEndian<double>(const double& data, std::vector<uint8_t>& array, size_t& write_pos);


//! template function for reating data out of an array while converting everything into correct endianess
template <typename T>
size_t fromLittleEndian(T& data, std::vector<uint8_t>& array, size_t& read_pos)
{
  // TODO: Remove once everything is tested :)
  //std::cout << "From Little Endian Called with: "<<" Size_T: "<< sizeof(T) << " Pos: " << (int)read_pos << "Current Array Size: "<< (int)array.size() << std::endl;

  // Reset data as we only write with or
  data = 0;

  // Check if ArrayBuilder has enough data
  if (read_pos + sizeof(T) > array.size())
  {
    // TODO: better error handling?
    return read_pos;
  }


  // Endianess Conversion
  for (size_t i= 0; i< sizeof(T);++i)
  {
    // Copy each byte into the bytearray, always convert byte order back from little endian
    data |= (array[read_pos+i]& 0xFF) <<(i*8);
    //std::cout << "Converting Value: 0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>((array[read_pos+i]& 0xFF) <<(i*8)) << " At the position i="<<static_cast<int>(i) << "resulting in the variable data: " << static_cast<int>(data)  << std::dec <<" Or on DEC :" << static_cast<int>(data) << std::endl;
  }

  // Note: The Vector still contains the elements at this point maybe we would like to delete that? But its expensive
  return read_pos + sizeof(T);
}

//! Template specialization for float as these have to be handled seperately
template <>
ICL_COMM_IMPORT_EXPORT
size_t fromLittleEndian<float>(float& data, std::vector<uint8_t>& array, size_t& read_pos);

//! Template specialization for float as these have to be handled seperately
template <>
ICL_COMM_IMPORT_EXPORT
size_t fromLittleEndian<double>(double& data, std::vector<uint8_t>& array, size_t& read_pos);

//! template class holding an array and the current index for write commands. Can be used to easily create an array for low level byte streams
//!
//! The arraybuilder is intended to  be used as a conversion queue for low level byte streams.
//! Arbitrary data can be written into the arraybuider by calling the << operator. During this write operation the data
//! will be convertet into a little endian representation regardles of the source architecture.
//! This can be used when a Little Endian byte stream (i.e. for a serial device) is composed of several data types.
//! Example: To send 1 int and to floats via bytestream in little endian ending:
//! ArrayBuilder ab;
//! ab << int << float1 << float2;
//! send(ab.array);
//! To read out values from the arraybuilder you can simply use the << operator. This will automatically convert the little endian representation
//! to the byte order of the host system.
//!
//! NOTE: As the arraybuilder converts the data during write and read operations
//!       the raw data that is read out from the stream has to be appended without any conversion (use appendWithoutConversion). (TODO: Is there a better Solution to this?)
class ArrayBuilder
{
public:
  ArrayBuilder(size_t array_size = 1) :
    write_pos(0),
    read_pos(0),
    array(array_size, 0)
  { }

  //! current write position in array
  size_t write_pos;

  //! current read position in array
  size_t read_pos;

  //! array of template type TArray
  std::vector<uint8_t>  array;


  //!
  //! \brief Resets the Arraybuilder to initial state, all values will be deleted
  //! \param array_size size the array is supposed to have after reset
  void reset(size_t array_size = 1);

  //! add data without any byte conversion
  template <typename T>
  void appendWithoutConversion(const T& data)
  {
    // Resize the target array in case it it to small to avoid out of bounds acces
    if (write_pos + sizeof(T) > array.size())
    {
      array.resize(write_pos + sizeof(T) );
    }

    // write data to array without conversion
    *(reinterpret_cast<T*>(&array[write_pos])) = data;
    write_pos += sizeof(T);
  }

  //! add data in vectors without any byte conversion
  template <typename T>
  void appendWithoutConversion(const std::vector<T>& data)
  {
    // Just insert every element of the Vector individually
    for (typename std::vector<T>::const_iterator it = data.begin() ; it != data.end(); ++it)
    {
      appendWithoutConversion(*it);
    }
  }

  //! Write any type into ArrayBuilder, convert to LittleEndian in process
  template <typename T>
  ArrayBuilder& operator << (const T& data);

  //! Write vectors into ArrayBuilder, each element is written seperately and convert to LittleEndian in process
  template <typename T>
  ArrayBuilder& operator << (const std::vector<T>& data);


  //! Read a generic data type from the array builder, Endianess is converted to system architecture
  template <typename T>
  ArrayBuilder& operator >> (T& data);

  //! Read a vectors from the array builder, Endianess is converted to system architecture
  template <typename T>
  ArrayBuilder& operator >> (std::vector<T>& data);

  //! Read out the last data without removing it from the stream
  //! NOTE: This is only implemented for base types
  template <typename T>
  T readBack();
};


//! Write any type into ArrayBuilder, convert to LittleEndian in process
template <typename T>
ArrayBuilder& ArrayBuilder::operator << (const T& data)
{
  // Convert the type to correct encoding and poit it into the Array
  write_pos = toLittleEndian<T>(data, array, write_pos);

  // TODO: Remove debug output
  //std::cout << "Arraybuilder got a generic type of length: "<< sizeof(data) << " and With data Packet: "<< (int)data << std::endl;

  return *this;
}


//! Write vectors into ArrayBuilder, each element is written seperately and convert to LittleEndian in process
template <typename T>
ArrayBuilder& ArrayBuilder::operator << (const std::vector<T>& data)
{
  // Just insert every element of the Vector individually
  for (typename std::vector<T>::const_iterator it = data.begin() ; it != data.end(); ++it)
  {
    *this << *it;
  }

  // TODO: Remove Debug output
  //std::cout << "ArrayBuilder got a vector of length: "<< data.size() << std::endl;

  return *this;
}


//! Read arbitrary types from the arraybuilder, endianess is converted during readout
template <typename T>
ArrayBuilder& ArrayBuilder::operator >> (T& data)
{
  read_pos = fromLittleEndian<T>(data, array, read_pos);
  return *this;
}

//! Read a vectors from the array builder, Endianess is converted to system architecture, vector has to be initualized
template <typename T>
ArrayBuilder& ArrayBuilder::operator >> (std::vector<T>& data)
{
  // Todo: For u_int8 Vectors this could also just be handled by an insert -> faster :)
  // Just insert every element of the Vector individually --> Do it in reverse order as we read from the end of the list :)
  for (typename std::vector<T>::iterator it = data.begin() ; it != data.end(); ++it)
  {
    *this >> *it;
  }

 return *this;
}


template <typename T>
T ArrayBuilder::readBack()
{
  T data;
  size_t read_back_pos = write_pos-sizeof(T);
  fromLittleEndian<T>(data, array, read_back_pos);
  return data;
}


//! debug output of array as hex values
std::ostream& operator << (std::ostream& o, const ArrayBuilder& ab);

}

#endif
