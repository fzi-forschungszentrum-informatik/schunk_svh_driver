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
 * \author  Georg Heppner <heppner@fzi.de>
 * \date    2014-02-07
 *
 *
 */
//----------------------------------------------------------------------

# include <icl_comm/ByteOrderConversion.h>



namespace icl_comm {

std::ostream& operator << (std::ostream& o, const ArrayBuilder& ab)
{
  for (size_t i = 0; i < ab.array.size(); i++)
  {
    o << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(ab.array[i]) << " ";
  }
  // Reset Output stream to decimal output .. otherwise it may confuse people
  std::cout << std::dec ;
  return o;
}

template <>
size_t toLittleEndian<float>(const float& data, std::vector<uint8_t>& array, size_t& write_pos)
{
  //! As Bit Shifts for floats are dissallowed they have to be given as reinterpreted value to currectly shift each byte individually
  return toLittleEndian(*(reinterpret_cast<const u_int32_t*>(&data)),array,write_pos);
}

template <>
size_t toLittleEndian<double>(const double& data, std::vector<uint8_t>& array, size_t& write_pos)
{
  //! As Bit Shifts for doubles are dissallowed they have to be given as reinterpreted value to currectly shift each byte individually
  return toLittleEndian(*(reinterpret_cast<const u_int64_t*>(&data)),array,write_pos);

}

template <>
size_t fromLittleEndian<float>(float& data, std::vector<uint8_t>& array, size_t& read_pos)
{
  //! As Bit Shifts for floats are dissallowed they have to be given as reinterpreted value to currectly shift each byte individually
  return fromLittleEndian(*(reinterpret_cast<u_int32_t*>(&data)),array,read_pos);
}

template <>
size_t fromLittleEndian<double>(double& data, std::vector<uint8_t>& array, size_t& read_pos)
{
  //! As Bit Shifts for doubles are dissallowed they have to be given as reinterpreted value to currectly shift each byte individually
  return fromLittleEndian(*(reinterpret_cast<u_int64_t*>(&data)),array,read_pos);

}

void ArrayBuilder::reset(size_t array_size)
{
  array.clear();
  write_pos = 0;
  read_pos = 0;
  array.resize(array_size,0);
}


}
